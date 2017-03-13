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

typedef struct {
	pid left;
	pid right;
} drivebasePID;

void initPIDController (pid *controller, float kP, float kI, float kD, word threshold, word integralLimit) {
	controller->kP = kP;
	controller->kI = kI;
	controller->kD = kD;
	controller->threshold = threshold;
	controller->integralLimit = integralLimit;
}

void initPIDController (pid *controller, float kP, float kI, float kD, word threshold) {
	initPIDController(controller, kP, kI, kD, -1);
}

void initPIDController (pid *controller, float kP, float kI, float kD) {
	initPIDController(controller, kP, kI, kD, 10);
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
