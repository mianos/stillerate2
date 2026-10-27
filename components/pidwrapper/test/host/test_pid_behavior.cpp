// Behavior / edge / property tests for PidCore — complements the unit-level
// assertions in test_pidcore.cpp with:
//   * edge cases: dt<=0, very large dt, NaN-input contract
//   * invariants over randomized runs (output & integral always bounded; finite)
//   * asserted closed-loop tests across plant models (settle, overshoot,
//     disturbance rejection, setpoint tracking, stability incl. dead-time)
//   * resetDerivative bumpless behavior; derivative-filter attenuation
//
// Build & run via the test/host CMakeLists (ctest), or directly:
//   c++ -std=c++17 -I../../include test_pid_behavior.cpp -o test_pid_behavior && ./test_pid_behavior

#include "PidCore.h"

#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <string>

static int g_failures = 0, g_checks = 0;
#define CHECK(cond)                                                            \
    do { ++g_checks; if (!(cond)) { ++g_failures;                             \
        std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static void run(const char* name, void (*fn)()) {
    int before = g_failures;
    fn();
    std::printf("[%s] %s\n", g_failures == before ? "PASS" : "FAIL", name);
}

// Deterministic PRNG (reproducible across platforms — no rand()).
struct Rng {
    unsigned long long s = 88172645463325252ULL;
    double unit() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (s >> 11) * (1.0 / 9007199254740992.0); }
    double in(double lo, double hi) { return lo + (hi - lo) * unit(); }
};

// --- Configurable test controller -------------------------------------------
static PidCore make(double kp, double ki, double kd) {
    PidCore p;
    p.kp = kp; p.ki = ki; p.kd = kd;
    p.output_min = 0.0; p.output_max = 100.0;
    p.integral_limit = 500.0;
    p.integral_unwind_factor = 0.1;
    p.wind_down_threshold = 0.0;     // leak off unless a test enables it
    p.derivative_filter = 0.0;
    p.set_point = 50.0;
    return p;
}

// === Edge cases ==============================================================

// dt == 0 must not integrate, differentiate, or produce NaN.
static void test_dt_zero_is_safe() {
    PidCore p = make(2.0, 1.0, 5.0);
    p.compute(60.0, 1.0);                 // prime with a normal step
    double before = p.integral();
    PidResult r = p.compute(70.0, 0.0);   // zero dt
    CHECK(std::isfinite(r.output));
    CHECK(r.D == 0.0);                     // derivative guarded on dt>0
    CHECK(p.integral() == before);        // no accumulation at dt=0
}

// Negative dt (clock glitch) must also be inert, not explosive.
static void test_dt_negative_is_safe() {
    PidCore p = make(2.0, 1.0, 5.0);
    p.compute(60.0, 1.0);
    double before = p.integral();
    PidResult r = p.compute(70.0, -3.0);
    CHECK(std::isfinite(r.output));
    CHECK(p.integral() == before);
}

// A huge dt (scheduling stall) must clamp the integral, not blow up.
static void test_huge_dt_clamps() {
    PidCore p = make(0.0, 1.0, 0.0);
    p.integral_limit = 200.0;
    PidResult r = p.compute(60.0, 1.0e6);  // error*dt would be ~1e7
    CHECK(std::isfinite(r.output));
    CHECK(std::fabs(p.integral()) <= 200.0 + 1e-9);
    CHECK(r.output >= p.output_min && r.output <= p.output_max);
}

// Contract characterization: PidCore does NOT sanitize a NaN measurement — it
// propagates. This is intentional: the caller (PidControlTimer) rejects bad
// reads via isfinite() before calling. This test pins that contract so a future
// change either keeps it or deliberately updates this expectation.
static void test_nan_input_propagates_by_contract() {
    PidCore p = make(2.0, 1.0, 5.0);
    p.compute(50.0, 1.0);
    PidResult r = p.compute(std::numeric_limits<double>::quiet_NaN(), 1.0);
    CHECK(std::isnan(r.output));   // => caller MUST filter NaN (PidControlTimer does)
}

// === Invariants over randomized runs ========================================

// Across many random (but valid) gains/inputs/dt, the output must always stay
// within [min,max], the integral within +/-limit, and everything finite.
static void test_invariants_property() {
    Rng rng;
    for (int trial = 0; trial < 200; ++trial) {
        PidCore p = make(rng.in(0, 10), rng.in(0, 2), rng.in(0, 50));
        p.output_min = rng.in(0, 20);
        p.output_max = p.output_min + rng.in(10, 100);
        p.integral_limit = rng.in(10, 1000);
        p.wind_down_threshold = rng.in(0, 5);
        p.integral_unwind_factor = rng.in(0, 0.99);
        p.derivative_filter = rng.in(0, 0.9);
        p.set_point = rng.in(-10, 120);
        for (int step = 0; step < 40; ++step) {
            double meas = rng.in(-50, 250);
            double dt = rng.in(0.01, 20);
            PidResult r = p.compute(meas, dt);
            CHECK(std::isfinite(r.output));
            CHECK(std::isfinite(p.integral()));
            CHECK(r.output >= p.output_min - 1e-9 && r.output <= p.output_max + 1e-9);
            CHECK(std::fabs(p.integral()) <= p.integral_limit + 1e-6);
        }
    }
}

// === Closed-loop behavior (asserted) ========================================

// Self-regulating first-order plant: temp relaxes toward (base - cool*output)
// with time constant tau. Holds setpoint at output = (base - setpoint)/cool.
struct LagPlant {
    double temp, base, cool, tau;
    void step(double output, double dt) {
        double target = base - cool * output;
        double alpha = 1.0 - std::exp(-dt / tau);
        temp += (target - temp) * alpha;
    }
};

// Reverse-acting plant: the actuator ADDS heat, so temperature rises with
// output (the opposite sense to the reflux cooling loop). Regulating this needs
// reverse-acting control — negative gains, since error = measurement - set_point.
struct HeatPlant {
    double temp, ambient, heat_gain, tau;
    void step(double output, double dt) {
        double target = ambient + heat_gain * output;
        double alpha = 1.0 - std::exp(-dt / tau);
        temp += (target - temp) * alpha;
    }
};

// First-order plant with transport delay (dead time) — a harder, realistic case.
struct DeadTimePlant {
    double temp, base, cool, tau;
    int delay_steps;
    std::deque<double> pipe;
    double step(double output, double dt) {
        pipe.push_back(output);
        double effective = pipe.front();
        if ((int)pipe.size() > delay_steps) pipe.pop_front();
        double target = base - cool * effective;
        double alpha = 1.0 - std::exp(-dt / tau);
        temp += (target - temp) * alpha;
        return temp;
    }
};

// Well-damped PI gains for the lag plant used below. Leak left OFF: a leaky
// integrator deliberately trades windup protection for steady-state droop, so it
// cannot hold zero offset — these tests assert exact regulation. (The leak's
// time-based behavior is covered by test_pidcore's integral-leak test.)
static PidCore tunedPI() {
    PidCore p = make(2.0, 0.20, 0.0);
    p.wind_down_threshold = 0.0;
    p.integral_limit = 2000.0;   // generous: required integral must be reachable
    p.set_point = 50.0;
    return p;
}

static void test_step_response_settles_bounded_overshoot() {
    PidCore p = tunedPI();
    LagPlant plant{80.0, 90.0, 1.0, 30.0};   // starts hot, settles to sp=50 at out≈40
    const double dt = 5.0;
    double peak_below = plant.temp;
    int settle_step = -1;
    for (int i = 0; i < 400; ++i) {
        double out = p.compute(plant.temp, dt).output;
        plant.step(out, dt);
        if (plant.temp < peak_below) peak_below = plant.temp;
        if (settle_step < 0 && std::fabs(plant.temp - p.set_point) < 0.5) settle_step = i;
    }
    CHECK(settle_step >= 0);                       // it settles
    CHECK(std::fabs(plant.temp - p.set_point) < 0.5);  // and stays settled
    CHECK(p.set_point - peak_below < 8.0);         // overshoot below sp bounded (<8 of a 30 step)
}

static void test_zero_steady_state_error_with_integral() {
    PidCore p = tunedPI();
    LagPlant plant{50.0, 90.0, 1.0, 30.0};
    for (int i = 0; i < 600; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    CHECK(std::fabs(plant.temp - p.set_point) < 0.3);   // integral removes offset
}

static void test_p_only_has_offset() {
    PidCore p = make(1.5, 0.0, 0.0);   // no integral
    p.set_point = 50.0;
    LagPlant plant{50.0, 90.0, 1.0, 30.0};
    for (int i = 0; i < 600; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    CHECK(std::fabs(plant.temp - p.set_point) > 1.0);   // proportional droop remains
}

static void test_disturbance_rejection() {
    PidCore p = tunedPI();
    LagPlant plant{50.0, 90.0, 1.0, 30.0};
    for (int i = 0; i < 300; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    CHECK(std::fabs(plant.temp - p.set_point) < 0.5);   // settled
    plant.base += 25.0;                                 // load disturbance (more heat)
    for (int i = 0; i < 400; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    CHECK(std::fabs(plant.temp - p.set_point) < 0.5);   // rejected, back to setpoint
}

static void test_setpoint_tracking() {
    PidCore p = tunedPI();
    LagPlant plant{50.0, 90.0, 1.0, 30.0};
    for (int i = 0; i < 300; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    p.set_point = 35.0;                                 // new target
    for (int i = 0; i < 500; ++i) plant.step(p.compute(plant.temp, 5.0).output, 5.0);
    CHECK(std::fabs(plant.temp - 35.0) < 0.5);
}

// === Reverse-acting (heating) loop ==========================================
// Same controller, NEGATIVE gains, on a plant whose output adds heat. Verifies
// the controller is sign-symmetric — in particular that the back-calculation
// anti-windup is correct when ki < 0.

static void test_reverse_acting_heating_regulates() {
    PidCore p = make(-2.0, -0.2, 0.0);   // reverse-acting: negative gains
    p.wind_down_threshold = 0.0;
    p.integral_limit = 2000.0;
    p.set_point = 50.0;
    HeatPlant plant{20.0, 20.0, 1.0, 30.0};   // cold start; holds sp=50 at out≈30
    for (int i = 0; i < 600; ++i) {
        double out = p.compute(plant.temp, 5.0).output;
        CHECK(out >= p.output_min - 1e-9 && out <= p.output_max + 1e-9);
        plant.step(out, 5.0);
    }
    CHECK(std::fabs(plant.temp - p.set_point) < 0.5);   // regulates to setpoint, in reverse
}

// Anti-windup must work with ki < 0 too: drive the output to a rail, confirm the
// integrator stops winding up, then recovers immediately when the error flips.
static void test_reverse_acting_anti_windup() {
    PidCore p = make(0.0, -10.0, 0.0);   // reverse pure-I
    p.integral_limit = 1.0e9;            // ensure it's anti-windup, not the clamp
    p.set_point = 50.0;
    p.compute(45.0, 1.0);                // error -5 (below sp) -> output rises
    p.compute(45.0, 1.0);                // reaches the max rail
    double sat_integral = p.integral();
    CHECK(p.compute(45.0, 1.0).saturated);
    for (int i = 0; i < 50; ++i) p.compute(45.0, 1.0);
    CHECK(std::fabs(p.integral() - sat_integral) < 1e-9);   // no wind-up while pinned
    PidResult r = p.compute(55.0, 1.0);                     // error flips to +5
    CHECK(r.output < p.output_max);                         // comes off the rail at once
}

// With dead time, conservative gains must remain bounded/stable (no divergence).
static void test_dead_time_remains_stable() {
    PidCore p = make(0.6, 0.05, 0.0);
    p.wind_down_threshold = 0.0;
    p.integral_limit = 2000.0;
    p.set_point = 50.0;
    DeadTimePlant plant{80.0, 90.0, 1.0, 30.0, /*delay_steps=*/4, {}};
    double tmax = -1e9, tmin = 1e9;
    for (int i = 0; i < 800; ++i) {
        double out = p.compute(plant.temp, 5.0).output;
        double t = plant.step(out, 5.0);
        if (i > 100) { tmax = std::fmax(tmax, t); tmin = std::fmin(tmin, t); }
        CHECK(std::isfinite(t));
    }
    CHECK(tmax < 120.0 && tmin > -20.0);             // bounded, did not diverge
    CHECK(std::fabs(plant.temp - p.set_point) < 5.0); // and roughly on target
}

// === resetDerivative & derivative filter ====================================

// resetDerivative clears the derivative history but PRESERVES the integral
// (bumpless restart), unlike reset() which clears both.
static void test_reset_derivative_preserves_integral() {
    PidCore p = make(1.0, 1.0, 5.0);
    p.compute(60.0, 1.0);
    p.compute(62.0, 1.0);
    double saved = p.integral();
    CHECK(saved != 0.0);
    p.resetDerivative();
    CHECK(p.integral() == saved);                 // integral kept
    PidResult r = p.compute(70.0, 1.0);           // first sample after clear
    CHECK(r.D == 0.0);                            // derivative history was cleared
}

// The derivative filter must attenuate the derivative response to a noisy signal.
static void test_derivative_filter_attenuates() {
    auto peakD = [](double filter) {
        PidCore p = make(0.0, 0.0, 10.0);
        p.derivative_filter = filter;
        double peak = 0.0;
        for (int i = 0; i < 60; ++i) {
            double meas = 50.0 + ((i % 2) ? 1.0 : -1.0);  // +/-1 square noise
            double d = p.compute(meas, 1.0).D;
            if (i > 2) peak = std::fmax(peak, std::fabs(d));
        }
        return peak;
    };
    double unfiltered = peakD(0.0);
    double filtered = peakD(0.7);
    CHECK(filtered < unfiltered);
    CHECK(filtered > 0.0);
}

int main() {
    run("dt == 0 is safe", test_dt_zero_is_safe);
    run("dt < 0 is safe", test_dt_negative_is_safe);
    run("huge dt clamps integral", test_huge_dt_clamps);
    run("NaN input propagates (caller-filters contract)", test_nan_input_propagates_by_contract);
    run("invariants hold over randomized runs", test_invariants_property);
    run("step response settles, bounded overshoot", test_step_response_settles_bounded_overshoot);
    run("zero steady-state error with integral", test_zero_steady_state_error_with_integral);
    run("P-only leaves steady-state offset", test_p_only_has_offset);
    run("disturbance rejection", test_disturbance_rejection);
    run("setpoint tracking", test_setpoint_tracking);
    run("dead-time plant remains stable", test_dead_time_remains_stable);
    run("reverse-acting heating loop regulates", test_reverse_acting_heating_regulates);
    run("reverse-acting anti-windup (ki<0)", test_reverse_acting_anti_windup);
    run("resetDerivative preserves integral", test_reset_derivative_preserves_integral);
    run("derivative filter attenuates", test_derivative_filter_attenuates);

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
