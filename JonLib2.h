#pragma systemFile

#pragma platform(VEX2)
#pragma competitionControl(competition)

#include "Includes/VEX_Competition_Includes_JON.c"

#define MOVE_TIMEOUT 1000 //timeout for "auto" moves
#define THRESHOLD_COEFF 2 //expanded size of threshold for timeout

#include "Includes/PID.h"
#include "Includes/Movement.h"
#include "Includes/Gyro.h"