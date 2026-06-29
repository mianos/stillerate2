#pragma once
//
// PidCore — the pure numerical core of the reflux-still PID controller.
//
// Deliberately free of ESP-IDF / JSON / NVS / FreeRTOS dependencies so it can
// be unit-tested on the host (see components/pidwrapper/test/host). PIDController
// wraps this with persistence, JSON telemetry and a mutex.
//
// Convention (direct-acting cooling loop): error = measurement - set_point.
// A measurement above setpoint (too hot) yields a positive error and, with
// kp > 0, a larger output -> more reflux cooling.
//
#include <cmath>
#include <algorithm>

struct PidResult {
    double output{0.0};     // commanded output after saturation
    double error{0.0};
    double P{0.0};
    double I{0.0};
    double D{0.0};
    double integral{0.0};   // committed integral state (units: error*seconds)
    double dt{0.0};
    bool   saturated{false};
};

class PidCore {
public:
    // --- Tunable gains / limits (copied in from PIDController before each step) ---
    double kp{0.0}, ki{0.0}, kd{0.0};
    double output_min{0.0}, output_max{100.0};
    double integral_limit{100.0};        // hard clamp on the integral state
    double integral_unwind_factor{0.1};  // per-second leak fraction near setpoint
    double wind_down_threshold{4.0};     // |error| below which the integral leaks
    double derivative_filter{0.0};       // EMA coeff in [0,1) for D; 0 = unfiltered
    double set_point{0.0};

    // Full reset: clears integral and all derivative history.
    void reset() {
        integral_ = 0.0;
        resetDerivative();
    }

    // Clear only the derivative history. Use when (re)starting the loop so the
    // first post-restart sample does not produce a spurious derivative spike,
    // while preserving the learned integral (bumpless-ish restart).
    void resetDerivative() {
        prev_measurement_ = 0.0;
        d_filtered_ = 0.0;
        have_prev_ = false;
    }

    double integral() const { return integral_; }

    // One control step. `dt` is the elapsed time in seconds since the previous
    // call. Returns the commanded output and the term breakdown for telemetry.
    PidResult compute(double measurement, double dt) {
        PidResult r;
        r.dt = dt;

        const double error = measurement - set_point;
        r.error = error;

        // --- Proportional ---
        const double P = kp * error;

        // --- Derivative on measurement (avoids setpoint "derivative kick") ---
        // With error = measurement - set_point and a constant setpoint, this is
        // identical to derivative-on-error; on a setpoint change it does NOT
        // spike, because it never differentiates the setpoint.
        double D = 0.0;
        if (dt > 0.0 && have_prev_) {
            const double raw = kd * (measurement - prev_measurement_) / dt;
            if (derivative_filter > 0.0 && derivative_filter < 1.0) {
                d_filtered_ = derivative_filter * d_filtered_ +
                              (1.0 - derivative_filter) * raw;
                D = d_filtered_;
            } else {
                D = raw;
            }
        }
        prev_measurement_ = measurement;
        have_prev_ = true;

        // --- Integral candidate (sample-rate independent) ---
        // Accumulate the time-integral of the error, then apply a time-based
        // leak when we are close to setpoint, then hard-clamp the state.
        double candidate = integral_;
        if (dt > 0.0) {
            candidate += error * dt;
            if (std::fabs(error) < wind_down_threshold) {
                candidate *= std::pow(1.0 - integral_unwind_factor, dt);
            }
        }
        // --- Tentative output with the candidate integral ---
        double I = ki * candidate;
        double u = P + I + D;
        double u_sat = std::clamp(u, output_min, output_max);
        const bool saturated = (u != u_sat);   // did the controller demand exceed a rail?

        // --- Anti-windup by back-calculation ---
        // When the output saturates, hold the integrator at exactly the level
        // that keeps the output on the rail (so ki*integral == u_sat - P - D).
        // This lets the output reach the limit, stops the integrator from winding
        // up beyond it, and lets it unwind the instant the error reverses. With
        // ki == 0 there is no integrator to correct.
        if (saturated && ki != 0.0) {
            candidate = (u_sat - P - D) / ki;
        }

        // Hard safety clamp on the integral state, then commit.
        candidate = std::clamp(candidate, -integral_limit, integral_limit);
        integral_ = candidate;

        // --- Final output from the committed integral ---
        I = ki * integral_;
        u = P + I + D;
        u_sat = std::clamp(u, output_min, output_max);

        r.P = P;
        r.I = I;
        r.D = D;
        r.integral = integral_;
        r.output = u_sat;
        r.saturated = saturated;   // reflects the pre-saturation demand
        return r;
    }

private:
    double integral_{0.0};
    double prev_measurement_{0.0};
    double d_filtered_{0.0};
    bool   have_prev_{false};
};
