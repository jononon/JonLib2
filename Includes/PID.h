#pragma systemFile

typedef struct {
	float kP;
	float kI;
	float kD;
	int target;
	int error;
	int integral;
	int derivative;
	int lastError;
	int integralLimit;
	int threshold;
} pid;

void initPIDController (pid *controller, float kP, float kI, float kD, int threshold = 10, int integralLimit = -1) {
	controller->kP = kP;
	controller->kI = kI;
	controller->kD = kD;
	controller->target = 0;
	controller->error = 0;
	controller->integral = 0;
	controller->derivative = 0;
	controller->lastError = 0;
	controller->threshold = threshold;
	controller->integralLimit = integralLimit;
}

float updatePIDController (pid *controller, tSensors sensor) {
	controller->error = controller->target - SensorValue[sensor];

	controller->integral = controller->integral + controller->error;

	if(controller->integral == 0)
		controller->integral = 0;
	if(controller->integralLimit != -1 && abs(controller->error)>controller->integralLimit)
		controller->integral = 0;

	controller->derivative = controller->error - controller->lastError;
	controller->lastError = controller->error;

	return controller->kP*controller->error + controller->kI*controller->integral + controller->kD*controller->derivative;
}

void addTarget(pid *controller, int target) {
	controller->target = controller->target+target;
}

void setTarget(pid *controller, int target) {
	controller->target = target;
}

void setThreshold(pid *controller, int threshold) {
	controller->threshold = threshold;
}

void setIntegralLimit(pid *controller, int integralLimit) {
	controller->integralLimit = integralLimit;
}

void clearIntegral(pid *controller) {
	controller->integral = 0;
}

void resetSensor (tSensors sensor, int times) {
	SensorValue[sensor] = 0;
	if(times>0)
		resetSensor(sensor, times-1);
}

void resetSensor (tSensors sensor) {
	resetSensor(sensor,3);
}
