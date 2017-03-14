#pragma systemFile

void setLeftWheelSpeed (int speed = 127);
void setRightWheelSpeed (int speed = 127);

void setWheelSpeed (int leftWheelSpeed = 127, int rightWheelSpeed = 127) {
	setLeftWheelSpeed(leftWheelSpeed);
	setRightWheelSpeed(rightWheelSpeed);
}

void setWheeelSpeed (int speed = 127) {
	setWheelSpeed(speed, speed);
}

void spin (int speed = 127) {
	setWheelSpeed(-speed, speed);
}

void timeDrive(int time, int leftPower = 127, int rightPower = 127) {
	setWheelSpeed(leftPower, rightPower);
	delay(time);
}

void timeDriveStop(int time, int leftPower = 127, int rightPower  = 127) {
	timeDrive(time, leftPower, rightPower);
	setWheelSpeed(0,0);
}

void tankDrive(int leftPower, int rightPower, int deadbands) {
	setWheelSpeed(
		abs(leftPower)<deadbands?0:leftPower,
		abs(rightPower)<deadbands?0:rightPower
	);
}

typedef struct {
	pid left;
	pid right;
} drivebase;

void initPIDDrivebase (drivebase *controller, float kP,  float kI, float kD, word threshold = 10, word integralLimit = -1) {
	initPIDController(controller->left,  kP, kI, kD, threshold, integralLimit);
	initPIDController(controller->right, kP, kI, kD, threshold,  integralLimit);
}

//todo - make these timeout + return false
void addMoveTargetPID(drivebase *controller,  int leftTarget, int rightTarget) {
	addTarget(controller->left, leftTarget);
	addTarget(controller->right, rightTarget);
}

void addMoveTargetPID(drivebase *controller, int target) {
	addMoveTargetPID(controller, target, target);
}

bool addMoveTargetPIDAuto(drivebase *controller, int leftTarget, int rightTarget) {
	addMoveTargetPID(controller, leftTarget, rightTarget);
	pid *left = controller->left;
	pid *right = controller->right;
	while(left->error<=left->threshold && right->error<=right->threshold)
		delay(25);
	return true;
}

bool addMoveTargetPIDAuto(drivebase *controller, int target) {
	addMoveTargetPID(controller, target);
	pid *left = controller->left;
	pid *right = controller->right;
	while(left->error<=left->threshold && right->error<=right->threshold)
		delay(25);
	return true;
}

void setMoveTargetPID(drivebase *controller,  int leftTarget, int rightTarget) {
	setTarget(controller->left, leftTarget);
	setTarget(controller->right, rightTarget);
}

void setMoveTargetPID(drivebase *controller, int target) {
	setMoveTargetPID(controller, target, target);
}

bool setMoveTargetPIDAuto(drivebase *controller, int leftTarget, int rightTarget) {
	setMoveTargetPID(controller, leftTarget, rightTarget);
	pid *left = controller->left;
	pid *right = controller->right;
	while(left->error<=left->threshold && right->error<=right->threshold)
		delay(25);
	return true;
}

bool setMoveTargetPIDAuto(drivebase *controller, int target) {
	setMoveTargetPID(controller, target);
	pid *left = controller->left;
	pid *right = controller->right;
	while(left->error<=left->threshold && right->error<=right->threshold)
		delay(25);
	return true;
}
