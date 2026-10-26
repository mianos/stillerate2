// Host tests for PIDController — the JSON/NVS layer on top of PidCore. Runs the
// REAL PIDController against real cJSON/JsonWrapper, a stub in-memory NVS, and a
// stub esp_log. Covers the integral_unwind_factor validation and the NVS
// persistence round-trip.
//
// Build & run (requires managed_components present, i.e. after one idf.py build):
//   ./run_pidcontroller_test.sh
// or see that script for the exact compile line.

#include "esp_log.h"            // stub macros — must precede PIDController.h
#include "NvsStorageManager.h"  // stub in-memory NVS (via -Istubs)
#include "PIDController.h"       // real

#include <cmath>
#include <cstdio>

static int g_failures = 0, g_checks = 0;
#define CHECK(cond)                                                            \
    do { ++g_checks; if (!(cond)) { ++g_failures;                             \
        std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b)                                                       \
    do { ++g_checks; if (std::fabs((a) - (b)) > 1e-9) { ++g_failures;          \
        std::printf("  FAIL %s:%d  %.9g != %.9g\n", __FILE__, __LINE__,        \
                    (double)(a), (double)(b)); } } while (0)

// Build a single-field params doc.
static JsonWrapper oneField(const char* key, double v) {
    JsonWrapper j(cJSON_CreateObject());
    j.AddItem(std::string(key), v);
    return j;
}

static void run(const char* name, void (*fn)()) {
    int before = g_failures;
    fn();
    std::printf("[%s] %s\n", g_failures == before ? "PASS" : "FAIL", name);
}

// Valid integral_unwind_factor in [0,1) is accepted.
static void test_valid_unwind_accepted() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    CHECK(pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 0.3)));
    CHECK_NEAR(pid.integral_unwind_factor, 0.3);
    CHECK(pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 0.0)));
    CHECK_NEAR(pid.integral_unwind_factor, 0.0);
    CHECK(pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 0.99)));
    CHECK_NEAR(pid.integral_unwind_factor, 0.99);
}

// Out-of-range values are rejected; the previous valid value is kept.
static void test_out_of_range_rejected() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 0.25));  // good baseline
    std::printf("  (expect two E-logs below: 1.5 and -0.2 rejected)\n");

    // >= 1.0 -> rejected (would make pow base <= 0)
    pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 1.5));
    CHECK_NEAR(pid.integral_unwind_factor, 0.25);
    // < 0 -> rejected
    pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", -0.2));
    CHECK_NEAR(pid.integral_unwind_factor, 0.25);
    // exactly 1.0 -> rejected (half-open interval)
    pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 1.0));
    CHECK_NEAR(pid.integral_unwind_factor, 0.25);
}

// The whole point: after a rejected bad factor, compute() still yields a finite
// output (no NaN poisoning of the integral).
static void test_compute_stays_finite_after_bad_factor() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    pid.setParametersFromJsonWrapper(oneField("ki", 0.2));
    pid.setParametersFromJsonWrapper(oneField("set_point", 15.0));
    pid.setParametersFromJsonWrapper(oneField("wind_down_threshold", 4.0));
    pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 9.0));  // rejected
    double out = 0.0;
    for (int i = 0; i < 20; ++i) {
        pid.compute(15.5, 5.0, out);   // near setpoint -> leak path exercised
        CHECK(std::isfinite(out));
    }
}

// Valid params survive a save/load round-trip through (stub) NVS.
static void test_persistence_round_trip() {
    NvsStorageManager nvs;
    {
        PIDController pid(nvs);
        pid.setParametersFromJsonWrapper(oneField("kp", 2.0));
        pid.setParametersFromJsonWrapper(oneField("ki", 0.2));
        pid.setParametersFromJsonWrapper(oneField("integral_unwind_factor", 0.3));
    }
    PIDController reloaded(nvs);   // constructor calls loadParameters()
    CHECK_NEAR(reloaded.kp, 2.0);
    CHECK_NEAR(reloaded.ki, 0.2);
    CHECK_NEAR(reloaded.integral_unwind_factor, 0.3);
}

// Reverse-acting tuning over the MQTT/JSON path: negative gains must be accepted
// (no validation rejects them), survive a save/load round-trip, and actually
// produce reverse control action through PIDController::compute.
static void test_reverse_gains_via_json() {
    NvsStorageManager nvs;
    {
        PIDController pid(nvs);
        JsonWrapper j(cJSON_CreateObject());
        j.AddItem(std::string("kp"), -2.0);
        j.AddItem(std::string("ki"), 0.0);
        j.AddItem(std::string("kd"), 0.0);
        j.AddItem(std::string("set_point"), 50.0);
        CHECK(pid.setParametersFromJsonWrapper(j));
        CHECK_NEAR(pid.kp, -2.0);   // negative gain accepted, not rejected/clamped

        // Reverse action: hotter-than-setpoint -> LOWER output; colder -> higher.
        double hot = 1.0, cold = 1.0;
        pid.compute(60.0, 1.0, hot);    // error +10, P = -20 -> clamps low
        pid.compute(40.0, 1.0, cold);   // error -10, P = +20 -> output 20
        CHECK(cold > hot);
    }
    // Negative gains persist through (stub) NVS, sign intact.
    PIDController reloaded(nvs);
    CHECK_NEAR(reloaded.kp, -2.0);
}

int main() {
    run("reverse-acting gains via JSON/MQTT path", test_reverse_gains_via_json);
    run("valid unwind factor accepted", test_valid_unwind_accepted);
    run("out-of-range unwind factor rejected", test_out_of_range_rejected);
    run("compute stays finite after bad factor", test_compute_stays_finite_after_bad_factor);
    run("params persist across save/load", test_persistence_round_trip);
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
