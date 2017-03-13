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
