#pragma once
#include "JsonWrapper.h"


struct Emulation {
    bool enabled = false;
    double temp = 0.0;

    // Deserialize the Emulation data from a JsonWrapper
    bool fromJsonWrapper(const JsonWrapper& json) {
        bool updated = false;
        updated |= json.GetField("enabled", enabled, true);	// mandatory means only set if set
        updated |= json.GetField("temp", temp, true);
        return updated;
    }

    // Serialize the Emulation data to a JsonWrapper
    JsonWrapper toJsonWrapper() const {
        JsonWrapper json(cJSON_CreateObject());
        json.AddItem("enabled", enabled);
        json.AddItem("temp", temp);
        return json;
    }
};


