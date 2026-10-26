#pragma once
// Host stub for NvsStorageManager — an in-memory key/value store with the same
// API as the real (NVS-backed) component, so PIDController persistence can be
// exercised on the host.
#include <map>
#include <string>

class NvsStorageManager {
public:
    bool store(const std::string& key, const std::string& value) {
        store_[key] = value;
        return true;
    }
    bool retrieve(const std::string& key, std::string& value) const {
        auto it = store_.find(key);
        if (it == store_.end()) return false;
        value = it->second;
        return true;
    }
    bool clear(const std::string& key) { return store_.erase(key) > 0; }
    bool clearAll() { store_.clear(); return true; }

private:
    std::map<std::string, std::string> store_;
};
