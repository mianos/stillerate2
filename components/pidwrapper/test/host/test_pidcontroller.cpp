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

// Output limits are validated as a pair: 0 <= min <= max <= 100. A bad pair is
// rejected whole (the old pair stays), a good one is applied, and either field
// may arrive on its own and is checked against the other.
static void test_output_limits_validated() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    pid.setParametersFromJsonWrapper(oneField("output_min", 5.0));
    pid.setParametersFromJsonWrapper(oneField("output_max", 90.0));
    CHECK_NEAR(pid.output_min, 5.0);
    CHECK_NEAR(pid.output_max, 90.0);

    CHECK(!pid.setParametersFromJsonWrapper(oneField("output_min", 95.0)));   // > max
    CHECK(!pid.setParametersFromJsonWrapper(oneField("output_max", 120.0)));  // > 100
    CHECK(!pid.setParametersFromJsonWrapper(oneField("output_min", -1.0)));   // < 0
    CHECK_NEAR(pid.output_min, 5.0);
    CHECK_NEAR(pid.output_max, 90.0);

    JsonWrapper both(cJSON_CreateObject());    // reversed pair in one message
    both.AddItem("output_min", 60.0);
    both.AddItem("output_max", 40.0);
    CHECK(!pid.setParametersFromJsonWrapper(both));
    CHECK_NEAR(pid.output_min, 5.0);
    CHECK_NEAR(pid.output_max, 90.0);

    JsonWrapper ok(cJSON_CreateObject());      // valid pair swaps the range
    ok.AddItem("output_min", 40.0);
    ok.AddItem("output_max", 60.0);
    CHECK(pid.setParametersFromJsonWrapper(ok));
    CHECK_NEAR(pid.output_min, 40.0);
    CHECK_NEAR(pid.output_max, 60.0);
    CHECK_NEAR(pid.failSafeOutput(), 60.0);

    double out = 0.0;                          // output honours the new range
    pid.setParametersFromJsonWrapper(oneField("kp", 2.0));
    pid.setParametersFromJsonWrapper(oneField("set_point", 50.0));
    pid.compute(0.0, 5.0, out);
    CHECK_NEAR(out, 40.0);
    pid.compute(100.0, 5.0, out);
    CHECK_NEAR(out, 60.0);
}

// A negative integral_limit would invert PidCore's std::clamp bounds, so it is
// rejected and the previous value kept. Zero is legal (no integral authority).
static void test_negative_integral_limit_rejected() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    pid.setParametersFromJsonWrapper(oneField("integral_limit", 50.0));
    std::printf("  (expect one E-log below: integral_limit -5 rejected)\n");
    CHECK(!pid.setParametersFromJsonWrapper(oneField("integral_limit", -5.0)));
    CHECK_NEAR(pid.integral_limit, 50.0);
    CHECK(pid.setParametersFromJsonWrapper(oneField("integral_limit", 0.0)));
    CHECK_NEAR(pid.integral_limit, 0.0);
}

// Non-finite numbers arrive through overflow: cJSON parses 1e400 with strtod,
// which returns inf. An inf set_point would drive the loop to 0 % coolant with
// the column hot (and be persisted to NVS), so it must be rejected.
static void test_non_finite_rejected() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    pid.setParametersFromJsonWrapper(oneField("kp", 2.0));
    pid.setParametersFromJsonWrapper(oneField("set_point", 78.0));
    std::printf("  (expect three E-logs below: inf set_point, kp, ki)\n");

    JsonWrapper inf_sp = JsonWrapper::Parse("{\"set_point\": 1e400}");
    CHECK(!pid.setParametersFromJsonWrapper(inf_sp));
    CHECK_NEAR(pid.set_point, 78.0);

    double out = 0.0;                       // still commands coolant when hot
    pid.compute(85.0, 5.0, out);
    CHECK(std::isfinite(out));
    CHECK(out > 0.0);

    JsonWrapper inf_kp = JsonWrapper::Parse("{\"kp\": 1e400}");
    CHECK(!pid.setParametersFromJsonWrapper(inf_kp));
    CHECK_NEAR(pid.kp, 2.0);

    // A bad field does not veto the good ones in the same message.
    JsonWrapper mixed = JsonWrapper::Parse("{\"kp\": 3, \"ki\": 1e400}");
    CHECK(pid.setParametersFromJsonWrapper(mixed));
    CHECK_NEAR(pid.kp, 3.0);
    CHECK_NEAR(pid.ki, 0.0);
}

// The leak window and the derivative filter are range-checked like the unwind
// factor, so the reported value is always the one actually in force.
static void test_range_validated_fields() {
    NvsStorageManager nvs;
    PIDController pid(nvs);
    std::printf("  (expect three E-logs below: derivative_filter 1 and -0.5, "
                "wind_down_threshold -1)\n");

    CHECK(pid.setParametersFromJsonWrapper(oneField("derivative_filter", 0.6)));
    CHECK_NEAR(pid.derivative_filter, 0.6);
    CHECK(!pid.setParametersFromJsonWrapper(oneField("derivative_filter", 1.0)));
    CHECK(!pid.setParametersFromJsonWrapper(oneField("derivative_filter", -0.5)));
    CHECK_NEAR(pid.derivative_filter, 0.6);

    CHECK(pid.setParametersFromJsonWrapper(oneField("wind_down_threshold", 0.0)));
    CHECK_NEAR(pid.wind_down_threshold, 0.0);
    CHECK(!pid.setParametersFromJsonWrapper(oneField("wind_down_threshold", -1.0)));
    CHECK_NEAR(pid.wind_down_threshold, 0.0);
}

// Loading at boot must not write the parameters back (needless flash wear); a
// corrupt blob leaves the defaults in place instead of crashing.
static void test_load_does_not_rewrite_nvs() {
    NvsStorageManager nvs;
    {
        PIDController pid(nvs);
        pid.setParametersFromJsonWrapper(oneField("kp", 2.0));
    }
    const int writes_after_save = nvs.stores;
    CHECK(writes_after_save > 0);
    {
        PIDController reloaded(nvs);        // constructor loads
        CHECK_NEAR(reloaded.kp, 2.0);
    }
    CHECK(nvs.stores == writes_after_save);

    nvs.store("pid_params", "not json");
    PIDController broken(nvs);
    CHECK_NEAR(broken.kp, 0.0);
}

int main() {
    run("output limits validated as a pair", test_output_limits_validated);
    run("reverse-acting gains via JSON/MQTT path", test_reverse_gains_via_json);
    run("valid unwind factor accepted", test_valid_unwind_accepted);
    run("out-of-range unwind factor rejected", test_out_of_range_rejected);
    run("compute stays finite after bad factor", test_compute_stays_finite_after_bad_factor);
    run("params persist across save/load", test_persistence_round_trip);
    run("negative integral_limit rejected", test_negative_integral_limit_rejected);
    run("non-finite parameters rejected", test_non_finite_rejected);
    run("range-validated fields", test_range_validated_fields);
    run("load does not rewrite NVS", test_load_does_not_rewrite_nvs);
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
