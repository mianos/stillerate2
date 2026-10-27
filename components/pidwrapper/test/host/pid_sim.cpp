// Closed-loop emulator/simulator for the reflux PID.
//
// Drives the REAL PidCore (the exact code that runs on the device) through a
// simple first-order still model, so we can watch the controller actually
// regulate — something the on-device open-loop Emulation (fixed injected temp)
// can't show. Uses the gains observed live on still2.
//
// Build & run:
//   c++ -std=c++17 -I../../include pid_sim.cpp -o pid_sim && ./pid_sim
//
// Plant model: reflux temperature rises with boiler "drive" (heat) and falls
// with coolant pump duty (the controller output):
//     temp += (drive - cool_gain * output) * dt
// Equilibrium output = drive / cool_gain.

#include "PidCore.h"

#include <cmath>
#include <cstdio>
#include <string>

struct Plant {
    double temp;
    double drive;       // deg/s of heating
    double cool_gain;   // deg/s of cooling per % pump duty
    void step(double output, double dt) { temp += (drive - cool_gain * output) * dt; }
};

// Deterministic, reproducible "sensor noise" (no rand()).
static double noise(int i, double amp) {
    double s = std::sin(i * 12.9898) * 43758.5453;
    return amp * (2.0 * (s - std::floor(s)) - 1.0);
}

static PidCore still2() {
    PidCore p;
    p.kp = 2.0; p.ki = 0.2; p.kd = 30.0;       // live gains from still2
    p.output_min = 5.0; p.output_max = 100.0;
    p.integral_limit = 3000.0;
    p.integral_unwind_factor = 0.1;
    p.wind_down_threshold = 0.0;                // leak disabled (as configured)
    p.derivative_filter = 0.0;
    p.set_point = 15.0;
    return p;
}

struct Metrics { double overshoot_below, settle_t, final_temp, final_out; };

static Metrics run(const char* name, PidCore pid, Plant plant,
                   double dt, int steps, double noise_amp, int print_every,
                   double disturb_at = -1, double new_drive = 0) {
    std::printf("\n=== %s (dt=%.1fs, noise=±%.2f) sp=%.1f ===\n",
                name, dt, noise_amp, pid.set_point);
    std::printf("   t(s)   temp    out    err      P       I       D\n");
    Metrics m{0, -1, 0, 0};
    double min_temp = plant.temp;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        if (disturb_at >= 0 && std::fabs(t - disturb_at) < dt / 2) {
            plant.drive = new_drive;
            std::printf("   -- disturbance: drive -> %.2f at t=%.0fs --\n", new_drive, t);
        }
        double measured = plant.temp + noise(i, noise_amp);
        PidResult r = pid.compute(measured, dt);
        plant.step(r.output, dt);

        if (plant.temp < min_temp) min_temp = plant.temp;
        if (m.settle_t < 0 && std::fabs(plant.temp - pid.set_point) < 0.2) m.settle_t = t;
        if (i % print_every == 0 || i == steps - 1)
            std::printf("%7.0f %7.2f %6.1f %7.2f %7.2f %7.2f %7.2f\n",
                        t, plant.temp, r.output, r.error, r.P, r.I, r.D);
    }
    m.overshoot_below = pid.set_point - min_temp;
    m.final_temp = plant.temp;
    std::printf("  -> settle(<0.2)=%s  overshoot below sp=%.2f  final temp=%.2f\n",
                m.settle_t < 0 ? "never" : (std::to_string((int)m.settle_t) + "s").c_str(),
                m.overshoot_below, m.final_temp);
    return m;
}

int main() {
    const double dt = 5.0;  // still2 sample_time

    // A) Clean step response: start hot (25C), regulate to 15C.
    run("A. step response, clean sensor", still2(),
        Plant{25.0, 0.4, 0.01}, dt, 80, 0.0, 8);

    // B) Same, but with realistic RTD noise — exposes kd=30 amplification.
    run("B. step response, noisy sensor (kd=30 unfiltered)", still2(),
        Plant{25.0, 0.4, 0.01}, dt, 80, 0.15, 8);

    // C) Disturbance rejection: settle, then boiler power jumps (drive up).
    run("C. disturbance rejection", still2(),
        Plant{15.0, 0.4, 0.01}, dt, 100, 0.0, 12, /*disturb_at=*/250, /*new_drive=*/0.8);

    // D) kd filtered: same noisy case with derivative_filter=0.6.
    PidCore filt = still2(); filt.derivative_filter = 0.6;
    run("D. noisy sensor, derivative_filter=0.6", filt,
        Plant{25.0, 0.4, 0.01}, dt, 80, 0.15, 8);

    return 0;
}
