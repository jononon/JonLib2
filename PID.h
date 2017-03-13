#pragma sourceFile

typedef struct {
	float kP;
	float kI;
	float kD;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	word integralLimit;
	word threshold;
} pid;

void initPIDController (pid *controller, float kP, float kI, float kD, word threshold = 10, word integralLimit = -1) {
	controller->kP = kP;
	controller->kI = kI;
	controller->kD = kD;
	controller->threshold = threshold;
	controller->integralLimit = integralLimit;
}

float updatePIDController (pid *controller, float sensor) {
	controller->error = controller->target - sensor;

	controller->integral = controller->integral + controller->error;

	if(controller->integral == 0)
		controller->integral = 0;
	if(controller->integralLimit != -1 && abs(controller->error)>controller->integralLimit)
		controller->integral = 0;

	controller->derivative = controller->error - controller->lastError;
	controller->lastError = controller->error;

	return controller->kP*controller->error + controller->kI*controller->integral + controller->kD*controller->derivative;
}

void addTarget(pid *controller, float target) {
	controller->target = controller->target+target;
}

void setTarget(pid *controller, float target) {
	controller->target = target;
}

void setThreshold(pid *controller, word threshold) {
	controller->threshold = threshold;
}

void setIntegralLimit(pid *controller, word integralLimit) {
	controller->integralLimit = integralLimit;
}
