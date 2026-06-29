// Host unit tests for PidCore — the pure PID math, no ESP-IDF dependencies.
//
// Build & run:
//   cd components/pidwrapper/test/host
//   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
// or directly:
//   c++ -std=c++17 -I../../include test_pidcore.cpp -o test_pidcore && ./test_pidcore
//
// Each test targets a specific, theory-grounded property of the controller.
// Convention under test: error = measurement - set_point (direct-acting cooling).

#include "PidCore.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++g_checks;                                                            \
        if (!(cond)) {                                                         \
            ++g_failures;                                                      \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                      \
    } while (0)

static bool approx(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}
#define CHECK_NEAR(a, b, eps)                                                  \
    do {                                                                       \
        ++g_checks;                                                            \
        if (!approx((a), (b), (eps))) {                                        \
            ++g_failures;                                                      \
            std::printf("  FAIL %s:%d  %s ~= %s  (%.9g vs %.9g)\n", __FILE__,  \
                        __LINE__, #a, #b, (double)(a), (double)(b));           \
        }                                                                      \
    } while (0)

static void run(const char* name, void (*fn)()) {
    int before = g_failures;
    fn();
    std::printf("[%s] %s\n", g_failures == before ? "PASS" : "FAIL", name);
}

// A controller with the integral leak disabled, so integral tests are clean.
static PidCore make(double kp, double ki, double kd) {
    PidCore p;
    p.kp = kp; p.ki = ki; p.kd = kd;
    p.output_min = 0.0; p.output_max = 100.0;
    p.integral_limit = 1e9;          // effectively off unless a test sets it
    p.wind_down_threshold = 0.0;     // |error| < 0 is never true -> no leak
    p.derivative_filter = 0.0;       // unfiltered derivative
    p.set_point = 50.0;
    return p;
}

// --- Proportional direction & clamping ---------------------------------------
// Hotter than setpoint must increase cooling; colder must clamp at the floor.
static void test_proportional_direction_and_clamp() {
    PidCore p = make(2.0, 0.0, 0.0);
    CHECK_NEAR(p.compute(55.0, 1.0).output, 10.0, 1e-9);  // error +5 -> 2*5
    CHECK_NEAR(p.compute(50.0, 1.0).output, 0.0, 1e-9);   // error 0
    CHECK_NEAR(p.compute(45.0, 1.0).output, 0.0, 1e-9);   // error -5 -> -10, clamped to min
}

static void test_output_limits_respected() {
    PidCore p = make(100.0, 0.0, 0.0);
    p.output_min = 10.0; p.output_max = 90.0;
    CHECK_NEAR(p.compute(60.0, 1.0).output, 90.0, 1e-9);  // huge +error -> clamp high
    CHECK_NEAR(p.compute(40.0, 1.0).output, 10.0, 1e-9);  // huge -error -> clamp low
}

// --- Integral: accumulation, sign, and time-integration ----------------------
static void test_integral_accumulates_over_time() {
    PidCore p = make(0.0, 1.0, 0.0);   // pure I
    // constant error of +2, dt=1: integral grows by 2 each step
    CHECK_NEAR(p.compute(52.0, 1.0).output, 2.0, 1e-9);
    CHECK_NEAR(p.compute(52.0, 1.0).output, 4.0, 1e-9);
    CHECK_NEAR(p.compute(52.0, 1.0).output, 6.0, 1e-9);
}

// Integration is of error*dt, not per-sample: one big step == many small steps.
static void test_integral_sample_rate_independent() {
    PidCore a = make(0.0, 1.0, 0.0);
    PidCore b = make(0.0, 1.0, 0.0);
    double ya = a.compute(52.0, 2.0).integral;          // one step, dt=2
    double yb = b.compute(52.0, 1.0).integral;
    yb        = b.compute(52.0, 1.0).integral;          // two steps, dt=1
    CHECK_NEAR(ya, yb, 1e-9);                            // both => integral 4
    CHECK_NEAR(ya, 4.0, 1e-9);
}

static void test_integral_hard_clamp() {
    PidCore p = make(0.0, 1.0, 0.0);
    p.integral_limit = 5.0;
    for (int i = 0; i < 100; ++i) p.compute(60.0, 1.0);  // error +10 every step
    CHECK_NEAR(p.integral(), 5.0, 1e-9);                 // clamped, not 1000
}

// --- Anti-windup: the headline fix -------------------------------------------
// While the output is pinned at the rail, the integrator must NOT keep growing;
// and on error reversal the output must leave the rail within one step (no lag).
static void test_antiwindup_no_integrator_growth_while_saturated() {
    PidCore p = make(0.0, 10.0, 0.0);   // pure I, strong
    p.integral_limit = 1e9;             // ensure it's anti-windup, not the clamp

    // Saturate: error +5, ki*integral hits 100 within a couple of steps.
    p.compute(55.0, 1.0);
    p.compute(55.0, 1.0);
    double sat_integral = p.integral();
    CHECK(p.compute(55.0, 1.0).saturated);   // pinned at output_max

    // Keep pushing into the rail for many steps; integral must not wind up.
    for (int i = 0; i < 50; ++i) p.compute(55.0, 1.0);
    CHECK_NEAR(p.integral(), sat_integral, 1e-9);

    // Reverse the error: a wound-up integrator would keep the output pinned for
    // many steps. With anti-windup it must drop off the rail immediately.
    PidResult r = p.compute(45.0, 1.0);      // error now -5
    CHECK(r.output < 100.0);
}

// Anti-windup must still allow the integrator to unwind (integrate the other
// way) while saturated, so recovery isn't blocked.
static void test_antiwindup_allows_unwind_at_rail() {
    PidCore p = make(0.0, 10.0, 0.0);
    p.compute(55.0, 1.0);
    p.compute(55.0, 1.0);                    // saturated high
    double before = p.integral();
    p.compute(45.0, 1.0);                    // error -5 pulls integral down
    CHECK(p.integral() < before);            // integration in the unwinding dir is allowed
}

// --- Derivative: on measurement, kick-free -----------------------------------
static void test_derivative_responds_to_measurement_rate() {
    PidCore p = make(0.0, 0.0, 5.0);
    p.compute(50.0, 1.0);                    // prime previous measurement
    PidResult r = p.compute(52.0, 1.0);      // +2 over 1s
    CHECK_NEAR(r.D, 10.0, 1e-9);             // kd * dMeas/dt = 5 * 2
}

static void test_no_derivative_kick_on_setpoint_change() {
    PidCore p = make(0.0, 0.0, 5.0);
    p.compute(50.0, 1.0);                    // prime; measurement steady at 50
    PidResult steady = p.compute(50.0, 1.0);
    CHECK_NEAR(steady.D, 0.0, 1e-9);         // no measurement change -> D 0

    // Now slam the setpoint. Derivative-on-error would spike; ours must not.
    p.set_point = 30.0;                      // error jumps +20
    PidResult r = p.compute(50.0, 1.0);      // measurement still 50
    CHECK_NEAR(r.D, 0.0, 1e-9);
}

static void test_first_sample_has_zero_derivative() {
    PidCore p = make(0.0, 0.0, 5.0);
    PidResult r = p.compute(99.0, 1.0);      // no previous sample yet
    CHECK_NEAR(r.D, 0.0, 1e-9);
}

// --- Integral leak (wind-down) is time-based ---------------------------------
// Near setpoint the integral leaks; the decay must depend on elapsed time, not
// on how many samples elapsed. Use error==0 to isolate the leak from accrual.
static void test_integral_leak_is_time_based() {
    auto preload = [](PidCore& p) {
        p.wind_down_threshold = 0.0;         // disable leak while loading
        for (int i = 0; i < 5; ++i) p.compute(60.0, 1.0);   // integral = 50
    };
    PidCore a = make(0.0, 1.0, 0.0);
    PidCore b = make(0.0, 1.0, 0.0);
    preload(a); preload(b);
    CHECK_NEAR(a.integral(), 50.0, 1e-9);

    a.wind_down_threshold = 1.0; a.integral_unwind_factor = 0.2;
    b.wind_down_threshold = 1.0; b.integral_unwind_factor = 0.2;

    // error = 0 (measurement == setpoint): no accrual, pure leak.
    a.compute(50.0, 2.0);                    // one step, dt=2
    b.compute(50.0, 1.0);                    // two steps, dt=1
    b.compute(50.0, 1.0);
    CHECK_NEAR(a.integral(), b.integral(), 1e-9);
    CHECK_NEAR(a.integral(), 50.0 * std::pow(0.8, 2.0), 1e-9);
}

// --- Reset -------------------------------------------------------------------
static void test_reset_clears_state() {
    PidCore p = make(0.0, 1.0, 5.0);
    p.compute(60.0, 1.0);
    p.compute(62.0, 1.0);
    CHECK(p.integral() != 0.0);
    p.reset();
    CHECK_NEAR(p.integral(), 0.0, 1e-9);
    PidResult r = p.compute(70.0, 1.0);      // first sample after reset
    CHECK_NEAR(r.D, 0.0, 1e-9);              // derivative history cleared
}

// --- A small closed-loop sanity check ----------------------------------------
// Simulate a trivial first-order cooling plant and confirm the controller
// settles near setpoint without sustained offset (integral action removes it).
static void test_closed_loop_settles_near_setpoint() {
    PidCore p = make(2.0, 0.5, 0.0);
    p.set_point = 50.0;
    p.wind_down_threshold = 0.0;             // no leak for this test

    double temp = 70.0;                      // start hot
    const double dt = 1.0;
    // Plant: heat input pushes temp up; cooling output pulls it down.
    // dTemp = (ambient_drive - k_cool * output) * dt, simple and stable.
    const double ambient_drive = 8.0;        // would settle at +ve error w/o I
    const double k_cool = 0.2;
    for (int i = 0; i < 400; ++i) {
        double out = p.compute(temp, dt).output;
        temp += (ambient_drive - k_cool * out) * dt;
    }
    CHECK_NEAR(temp, 50.0, 0.5);             // integral kills steady-state offset
}

int main() {
    run("proportional direction & clamp", test_proportional_direction_and_clamp);
    run("output limits respected", test_output_limits_respected);
    run("integral accumulates over time", test_integral_accumulates_over_time);
    run("integral sample-rate independent", test_integral_sample_rate_independent);
    run("integral hard clamp", test_integral_hard_clamp);
    run("anti-windup: no growth while saturated", test_antiwindup_no_integrator_growth_while_saturated);
    run("anti-windup: allows unwind at rail", test_antiwindup_allows_unwind_at_rail);
    run("derivative responds to measurement rate", test_derivative_responds_to_measurement_rate);
    run("no derivative kick on setpoint change", test_no_derivative_kick_on_setpoint_change);
    run("first sample has zero derivative", test_first_sample_has_zero_derivative);
    run("integral leak is time-based", test_integral_leak_is_time_based);
    run("reset clears state", test_reset_clears_state);
    run("closed loop settles near setpoint", test_closed_loop_settles_near_setpoint);

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
