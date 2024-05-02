#pragma once

struct MqttContext {
	class PIDController *pid;
	class Emulation *emu;
	class PIDControlTimer *ptimer;
};

