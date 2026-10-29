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
        ++stores;   // test hook: how many writes reached "flash"
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

    int stores{0};   // test hook (not part of the real class)

private:
    std::map<std::string, std::string> store_;
};

