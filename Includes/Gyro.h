#pragma systemFile

typedef struct {
	pid controller;
	tSensors sensor;
} gyroscope;

void initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, word threshold = 10, word integralLimit = -1) {
	pid *controller = gyroController->controller;
	initPIDController(controller, kP, kI, kD, threshold, integralLimit);
	gyroController->sensor = sensor;
}

void setGyroTargetPID (gyroscope *gyroController, float target) {
	pid *controller = gyroController->controller;
	controller->target = target;
}

void addGyroTargetPID (gyroscope *gyroController, float target) {
	pid *controller = gyroController->controller;
	controller->target = controller->target + target;
}

bool leftSwingTurnGyroPID (gyroscope *gyroController) {
	pid *controller = gyroController->controller;

	long lastUpdate = nPgmTime;

	while(controller->error>controller->threshold) {
		setLeftWheelSpeed(updatePIDController(controller, gyroController->sensor));

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
	long lastUpdate = nPgmTime;
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
	long lastUpdate = nPgmTime;
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
