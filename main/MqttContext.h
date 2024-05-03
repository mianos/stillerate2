#pragma once

struct MqttContext {
	class PIDController *pid;
	class Emulation *emu;
	class PIDControlTimer *ptimer;
	class MotorController *reflux_pump;
	class MotorController *condenser_pump;
};

