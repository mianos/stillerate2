#pragma once

struct MqttContext {
	class PIDController *pid;
	class Emulation *emu;
	class PIDControlTimer *ptimer;
	class RESTMotorController *reflux_pump;
	class RESTMotorController *condenser_pump;
	class SettingsManager *settings;
};

