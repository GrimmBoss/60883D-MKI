#include "auton.cpp"
#include "lvgl_functions.cpp"
#define AUTO_SWTICH_THRESH 90


void initialize(){
	initBarGraph();
	pros::Task brainDataTask(updateBarGraph_fn);
	liftPot.set_data_rate(5);
	lift.set_brake_mode(MOTOR_BRAKE_HOLD);

	//Set vision values
	vision.set_exposure(20);
	vision.set_signature(YELLOW_SIG_INDEX, &YELLOW_SIG);
}


void disabled(){}


void competition_initialize(){
	int8_t lastLeftX;
	while(true){
		int8_t leftX = master.get_analog(ANALOG_LEFT_X);
		if((leftX <= -AUTO_SWTICH_THRESH) && (lastLeftX > -AUTO_SWTICH_THRESH)){auton--;}
		else if((leftX >= AUTO_SWTICH_THRESH) && (lastLeftX < AUTO_SWTICH_THRESH)){auton++;}
		lastLeftX = leftX;
		
		switch(auton%AUTO_NUMBER){
			case 0: master.print(2, 0, "Win Point, %.2f            ", imu.get_heading()); break;
			case 1: master.print(2, 0, "Mogo Rush, %.2f            ", imu.get_heading()); break;
			case 2: master.print(2, 0, "Elims Vision, %.2f         ", imu.get_heading()); break;
			case 3: master.print(2, 0, "Skills, %.2f               ", imu.get_heading()); break;
			case 4: master.print(2, 0, "Nothing, %.2f              ", imu.get_heading()); break;
		}
		pros::delay(20);
	}
}


void autonomous(){
	switch(auton%AUTO_NUMBER){
		case 0: winpoint();		break;
		case 1: mogorush();		break;
		case 2: elimsVision(); break;
		case 3: skills();		break;
		case 4: empty();		break;
	}
}


void opcontrol(){
	int8_t lastLeftX;
	//set piston vals here
	bool liftManual = false;
	backClamp.set_value(pneumatics.backPistonToggle);
	liftSol1.set_value(pneumatics.liftSol1Toggle);
	liftSol2.set_value(pneumatics.liftSol2Toggle);
	while (true) {
		//Assign variables to joystick values for ease of use
		int8_t leftX = master.get_analog(ANALOG_LEFT_X);
		int8_t leftY = master.get_analog(ANALOG_LEFT_Y);
		int8_t rightX = master.get_analog(ANALOG_RIGHT_X);
		int8_t rightY = master.get_analog(ANALOG_RIGHT_Y);


		//Change auton value
		if((leftX <= -AUTO_SWTICH_THRESH) && (lastLeftX > -AUTO_SWTICH_THRESH)){auton--;}
		else if((leftX >= AUTO_SWTICH_THRESH) && (lastLeftX < AUTO_SWTICH_THRESH)){auton++;}
		lastLeftX = leftX;


		//Reset the IMU when the up button is called
		if(master.get_digital_new_press(DIGITAL_UP)){
            imu.reset();
            float iter = 0;
            while (imu.is_calibrating()) {
                master.print(2,0,"IMU calibrate: %.2f  ", iter/1000);
                iter += 20;
                pros::delay(20);
            }
		}


		//Display current autonomous on the controller
		switch(auton%AUTO_NUMBER){
			case 0: master.print(2, 0, "Win Point, %.2f            ", imu.get_heading()); break;
			case 1: master.print(2, 0, "Mogo Rush, %.2f            ", imu.get_heading()); break;
			case 2: master.print(2, 0, "Elims Vision, %.2f         ", imu.get_heading()); break;
			case 3: master.print(2, 0, "Skills, %.2f               ", imu.get_heading()); break;
			case 4: master.print(2, 0, "Nothing, %.2f              ", imu.get_heading()); break;
		}


		//Set drive motor speeds
		int rightDrive = leftY - rightX;
		frontright = rightDrive;
		midright = rightDrive;
		backright = rightDrive;

		int leftDrive = leftY + rightX;
		frontleft = leftDrive;
		midleft = leftDrive;
		backleft = leftDrive;


		//Adjust toggleable values
		if(master.get_digital_new_press(DIGITAL_A)){
			pneumatics.toggleLiftClamp();
		}
		if(master.get_digital_new_press(DIGITAL_B)){
			pneumatics.lockLiftClamp();
		}
		if(master.get_digital_new_press(DIGITAL_X)){
			pneumatics.toggleBackClamp();
		}
		if(master.get_digital_new_press(DIGITAL_Y)){
			liftManual =! liftManual;
			if(!liftManual){
				liftPID.setTarget(getClosestIndicator(liftPot.get_angle()));
			}
		}


		//Set conveyor speed
		if(master.get_digital(DIGITAL_R1)){conveyor.move_velocity(200);}
		else if(master.get_digital(DIGITAL_R2)){conveyor.move_velocity(-200);}
		else{conveyor.move_velocity(0);}


		//Lift control
		if(liftManual){
			if(master.get_digital(DIGITAL_L1)){lift = 127;}
			else if(master.get_digital(DIGITAL_L2)){lift = -127;}
			else{lift = 0;}
		}
		else{
			if(master.get_digital_new_press(DIGITAL_L1) && liftPID.targetIndicator < LIFTVALSIZE-1){
				liftPID.targetIndicator++;
				liftPID.target = liftTargets[liftPID.targetIndicator%LIFTVALSIZE];
			}
			else if(master.get_digital_new_press(DIGITAL_L2) && liftPID.targetIndicator > 0){
				liftPID.targetIndicator--;
				liftPID.target = liftTargets[liftPID.targetIndicator%LIFTVALSIZE];
			}
			liftPID.PID();
		}


		//Run the backup function (Used mainly for driver skills)
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
			platformBackClamp();
		}

		
		//Run the currently selected autonomous when B is pressed
		if(master.get_digital_new_press(DIGITAL_DOWN)){
			#if SKILLSONDOWN
				skills();
			#else
				autonomous();

			#endif
		}

		pros::delay(20);
	}
}
