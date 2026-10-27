#pragma once

// Everything the REST command handlers act on.
struct StillContext {
	class PIDController *pid;
	class Emulation *emu;
	class PIDControlTimer *ptimer;
	class RESTMotorController *reflux_pump;
	class RESTMotorController *condenser_pump;
	class SettingsManager *settings;
	class WiFiManager *wifiManager;
};
