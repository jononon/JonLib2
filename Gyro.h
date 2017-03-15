#pragma systemFile

#define MOVE_TIMEOUT 1000  //timeout for "auto" moves
#define THRESHOLD_COEFF 2 //expanded size of threshold for timeout function

void initPIDGyroscope (pid *controller, float kP,  float kI, float kD, word threshold = 10, word integralLimit = -1) {
	initPIDController(controller, kP, kI, kD, threshold, integralLimit);
}

void setGyroTargetPID (pid *controller, float target) {
	controller->target = target;
}

void addGyroTargetPID (pid *controller, float target) {
	controller->target = controller->target + target;
}

bool leftSwingTurnGyroPID (pid *controller, tSensors gyro) {

	long lastUpdate = nPgmTime;

	while(controller->error>controller->threshold) {
		setLeftWheelSpeed(updatePIDController(controller, gyro));

		if(abs(controller->error)>controller->threshold*THRESHOLD_COEFF)
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			return false;
		}

		delay(25);
	}
	setWheelSpeed(0);
	return true;
}

bool rightSwingTurnGyroPID (pid *controller, tSensors gyro) {
	long lastUpdate = nPgmTime;;
	while(controller->error>controller->threshold) {
		setRightWheelSpeed(updatePIDController(controller, gyro));

		if(abs(controller->error)>controller->threshold*THRESHOLD_COEFF)
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			return false;
		}

		delay(25);
	}
	setWheelSpeed(0);
	return true;
}

bool pointTurnGyroPID (pid *controller, tSensors gyro) {
	long lastUpdate = nPgmTime;;
	while(controller->error>controller->threshold) {
		spin(updatePIDController(controller, gyro));

		if(abs(controller->error)>controller->threshold*THRESHOLD_COEFF)
			lastUpdate = nPgmTime;

		if((nPgmTime-lastUpdate)>MOVE_TIMEOUT) {
			setWheelSpeed(0);
			return false;
		}

		delay(25);
	}
	setWheelSpeed(0);
	return true;
}
