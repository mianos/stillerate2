#pragma once
#include <atomic>
#include <cmath>

#include "JsonWrapper.h"

// Written by the web server, read by the PID task.
struct Emulation {
    std::atomic<bool> enabled{false};
    std::atomic<double> temp{0.0};

    // Applies whichever of "enabled" and "temp" are present and valid.
    bool fromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;
        bool e;
        if (json.GetField("enabled", e)) {
            enabled = e;
            updated = true;
        }
        double t;
        if (json.GetField("temp", t) && std::isfinite(t) && t > -50.0 && t < 300.0) {
            temp = t;
            updated = true;
        }
        return updated;
    }

    void addTo(JsonWrapper& json) const {
        json.AddItem("emulation", enabled.load());
        json.AddItem("emulation_temp", temp.load());
    }
};
