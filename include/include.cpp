#pragma once
#include "main.h"
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define AUTO_NUMBER 5
#define LIFTVALSIZE 4

//Global Variable Declaration
uint8_t auton = AUTO_NUMBER;
const float liftTargets[LIFTVALSIZE] = {46, 78, 135, 162.5};

//Declare the controller
pros::Controller master(CONTROLLER_MASTER);

//Declare motors
pros::Motor frontright(5,false);
pros::Motor midright(13, true);
pros::Motor backright(1, false);

pros::Motor frontleft(8, true);
pros::Motor midleft(9, false);
pros::Motor backleft(10, true);

pros::Motor lift(20, false);
pros::Motor conveyor(14, false);

//Declare sensors
pros::Rotation liftPot(12);
pros::Imu imu(19);
pros::Vision vision(18, VISION_ZERO_CENTER);

//Set vision signatures
pros::vision_signature_s_t YELLOW_SIG =
pros::Vision::signature_from_utility(1, -1, 1633, 816, -4031, -1901, -2966, 1.200, 0);

//Declare ADI ports
pros::ADIDigitalOut backClamp(3, LOW);
pros::ADIDigitalOut liftSol1(1, LOW);
pros::ADIDigitalOut liftSol2(2, HIGH);
