void initPIDGyroscope (pid *controller, float kP,  float kI, float kD, word threshold = 10, word integralLimit = -1) {
	initPIDController(controller, kP, kI, kD, threshold, integralLimit);
}

void setGyroTargetPID (pid *controller, float target) {
	controller->target = target;
}

void addGyroTargetPID(pid *controller, float target) {
	controller->target = controller->target + target;
}
