#pragma once
#include "PidController.h"
#include "Emulation.h"
#include "PidControlTimer.h"

struct MqttContext {
	PIDController *pid;
	Emulation *emu;
	PIDControlTimer *ptimer;
};

