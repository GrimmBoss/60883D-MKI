#include "PID.cpp"
#include "onErrorPID.cpp"
#include "robotClasses.cpp"
#define SKILLSONDOWN 1


void winpoint(){
    //Set the lift target to zero before intializing the lift task
    liftPID.setTarget(0);
    //Initialize the automatic lift control task and the run on error task
    pros::Task liftControl(liftPID_fn);
    pros::Task runOnError(onError_fn);
    //Create a Triangle object to be used later in the auto
    Triangle tri;

    //Ensure that the pistons are set correctly
    liftClampPassive();
    backClampUp();

    deploy();
    pros::delay(400);

    //Drive backwards and release the mogo so that turning is not affected
    drive.addErrorFunc(10, liftClampUp);
    drive.setPID(1);
    drive.move(backward, 12, 1, 100);

    //Turn to swerve towards the other alliance mogo
    drive.move(left, imuTarget(305), 1, 80);
    
    drive.swerve(forwardRight, 102, imuTarget(0), 3, 100, 100);
    drive.move(left, imuTarget(180), 1, 100);

    //Turn around to back into the mogo and pick it up for winpoint

    moveDriveVoltage(-6000);
    pros::delay(800);
    backClampDown();

    liftClampPassive();
    setConveyor(100);
    drive.swerve(forwardRight, 41, imuTarget(270), 1.6, 95, 80);

    //DO TRIANGLE FUNCTION TO MAKE THIS MORE CONSISTENT

    drive.addErrorFunc(27, setLiftTargetOne);
    drive.addErrorFunc(5, setLiftTargetThree);
    drive.swerve(backwardLeft, 30, imuTarget(180), 1.5, 81, 100);

    liftPID.setTarget(3);
    setConveyor(200);

    drive.setPID(1);
    drive.hardStop(forward, 27, 70, 4, 70);
    drive.swerve(forwardLeft, 40, imuTarget(90), 1.5, 70, 40);

    liftPID.setTarget(2);
    pros::delay(200);
    liftClampUp();
    pros::delay(200);
    liftPID.setTarget(3);

    setConveyor(200);
    drive.move(backward, 12, 1, 100);
    liftPID.setTarget(0);
    drive.move(left, imuTarget(270), 1, 100);

    //Get yellow middle
    //drive.setPID(5);
    //liftClampPassive();
    //drive.visionSwerve(YELLOW_SIG_INDEX, VISION_CLOSEST, 30, 3, 100, 100);

    //Remove tasks used during the run and clear the vector
    liftControl.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void mogorush(){
    //Set the lift target to zero before intializing the lift task
    liftPID.setTarget(0);
    //Initialize the automatic lift control task and the run on error task
    pros::Task liftControl(liftPID_fn);
    pros::Task runOnError(onError_fn);
    //Create the Triangle object to be used throughout the run
    Triangle tri;

    //Ensure that the pistons are set correctly
    liftClampPassive();
    backClampUp();

    //Swerve and pick up first yellow
    drive.setPID(4);
    drive.swerve(forwardRight, 50, imuTarget(15), 1.7, 80, 13);
    liftClampDown();

    drive.move(backward, 40, 10, 100);
    //Check what error is and if the bot hasnt moved do something else before turning to get the middle mogo

    
    //Remove tasks used during the run and clear the vector
    liftControl.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void elimsVision(){
    //Set the lift target to zero before intializing the lift task
    liftPID.setTarget(0);
    //Initialize the automatic lift control task and the run on error task
    pros::Task liftControl(liftPID_fn);
    pros::Task runOnError(onError_fn);
    //Create the Triangle object to be used throughout the run
    Triangle tri;

    //Ensure that the pistons are set correctly
    liftClampPassive();
    backClampUp();

    //Deploy the ring mech
    deploy();


    //Set standstill to the lowest viable amount
    drive.setStandStill(3);

    //Swerve and pick up the middle mogo.
    drive.setPID(4);
    liftClampPassive();
    drive.swerve(forwardLeft, 57, imuTarget(310), 1.8, 90, 100);


    drive.setStandStill(DEFAULT_STANDSTILL);

    //Back up with mogo in tow and then turn 180 
    drive.setPID(1);
    drive.move(backward, 42, 1.2, 100);
    drive.move(left, imuTarget(140), 1.2, 60);

    //Tilt the mogo outside of the field wall (OP strat)
    liftPID.setTarget(1);
    liftClampUp();
    drive.setPID(2);
    drive.setStandStill(3);
    drive.move(forward, 12, 1, 100);
    pros::delay(500);

    //Pick up the red alliance mogo with the back clamp
    drive.setPID(1);
    drive.move(right, imuTarget(270), 1, 100);
    liftPID.setTarget(0);


    moveDriveVoltage(-6000);
    pros::delay(700);
    backClampDown();
    pros::delay(125);

    //Use trigonometry to make sure the vision swerve doesnt cross the line
    findTri(tri, 22, 270);

    //Drive forward and turn to imu 0 in preparation for the yoink
    setConveyor(150);
    drive.setStandStill(DEFAULT_STANDSTILL);
    drive.move(forward, tri.hyp, 1, 100);
    drive.move(right, imuTarget(0), 1, 100);

    //Epic swerve to yoink the mogo
    drive.setPID(5);
    liftClampPassive();
    drive.setStandStill(3);
    drive.visionSwerve(YELLOW_SIG_INDEX, VISION_CLOSEST, 48-tri.b, 3, 100, 100);

    //Back up once more with mogo in tow
    drive.move(backward, 20, 3, 100);
    setConveyor(0);

    //Remove tasks used during the run and clear the vector
    liftControl.remove();
    runOnError.remove();
    onErrorVector.clear();
}


void skills(){
    //Set the lift target to zero before intializing the lift task
    liftPID.setTarget(0);
    //Initialize the automatic lift control task and the onError task
    pros::Task liftControl(liftPID_fn);
    pros::Task runOnError(onError_fn);
    //Create the Triangle object to be used throughout the run, also create a variable to simplify trigonometry
    Triangle tri;
    float turnVal;

    //Ensure that the pistons are set correctly
    liftClampPassive();
    backClampUp();

    //Deploy the ring mech
    deploy();

    //Drive backwards and pick up the mogo with the back clamp
    platformBackClamp();

    //Swerve and pick up the mogo.
    drive.setPID(4);
    liftClampPassive();
    drive.swerve(forwardRight, 86, imuTarget(100), 2.5, 60, 100);
    //setConveyor(200);
    drive.addErrorFunc(24, setLiftTargetThree);
    setConveyor(100);
    drive.swerve(forwardRight, 41, imuTarget(180), 1.85, 60, 100);
    drive.setPID(3);

    //Set turnVal to 70 for trig
    turnVal = 70;
    drive.move(left, imuTarget(turnVal), 1, 100);

    //Lower the lift and drop the mogo, causing the platform to balance.
    drive.setPID(2);
    drive.move(forward, 10, 1, 100);
    //setConveyor(0);
    liftPID.setTarget(2);
    pros::delay(700);
    liftClampUp();
    
    //Back up and turn to imu 180 in order to line up for picking up and dropping next yellow
    findTri(tri, 6, turnVal);
    setConveyor(0);
    drive.move(backward, tri.hyp, 1, 100);
    drive.setTemporaryPID(12,100,10,100,15,100,100,0);
    drive.move(right, imuTarget(180), 1, 100);
    
    //Drive into position to turn and pickup second yellow
    liftPID.setTarget(0);
    drive.setPID(1);
    setConveyor(150);
    drive.setSlew(30);
    drive.addErrorFunc(40, setSlewZero);
    drive.move(forward, 49 + tri.b, 3, 75);
    drive.setSlew(0);

    //Set turnVal to 296 for trig
    turnVal = 296;
    drive.move(right, imuTarget(turnVal), 1, 100);

    //Drive forward, clamp down, and lift the second yellow, then drop it on the red platform.
    Triangle secondaryTri;
    findTri(tri, 84, turnVal);
    findTri(secondaryTri, 77.3, turnVal);
    drive.addErrorFunc(57, liftClampPassive);
    drive.addErrorFunc(55, setLiftTargetThree);
    drive.move(forward, tri.hyp, 7, 42);
    liftPID.setTarget(2);
    pros::delay(200);
    liftClampUp();
    pros::delay(200);
    liftPID.setTarget(3);
    

    //////////////////////////////////////POST SECOND YELLOW DROP//////////////////////////////////////

    //Back away from the platform and turn towards the last alliance mogo
    drive.setPID(2);
    drive.move(backward, 6.7, 1, 100);
    setConveyor(0);
    liftPID.setTarget(0);
    drive.setPID(2);
    drive.move(left, imuTarget(180), 1, 80);

    //Drive forward and clamp down when 5 inchs away from the target
    drive.addErrorFunc(11, liftClampPassive);
    drive.move(forward, 34 + secondaryTri.b, 2, 50);
    moveDriveVoltage(6000);
    pros::delay(200);

    //Back up and turn back towards the platform
    findTri(tri, 7, 90+71);
    liftPID.setTarget(3);
    drive.setPID(2);
    drive.move(backward, tri.hyp, 1, 100);
    drive.setPID(3);
    drive.move(left, imuTarget(71), 1, 80);

    //Drive towards the platform and drop the mogo off
    setConveyor(200);
    drive.setPID(1);
    drive.move(forward, 94-tri.b, 5.5, 70);
    setConveyor(100);
    liftPID.setTarget(2);
    pros::delay(200);
    liftClampUp();
    pros::delay(200);
    liftPID.setTarget(3);

    ////////////////////////////////POST FIRST ALLIANCE DROP (180 DROP OFF)////////////////////////////////

    //Drive backwards and drop the back mogo
    drive.setStandStill(3);
    drive.setPID(2);
    drive.addErrorFunc(12, setLiftTargetZero);
	drive.move(backward, 17, 1, 100);
	backClampUp();
    pros::delay(100);
	drive.move(forward, 10, 1, 100);

    //Turn 180 degrees and pick the mogo up
	drive.setPID(1);
	drive.move(left, 180, 2, 100);
	drive.setPID(2);
    drive.addErrorFunc(4, liftClampPassive);
	drive.move(forward, 12, 1, 100);

    //Turn and drop the mogo on the platform
	liftPID.setTarget(3);
    drive.setTemporaryPID(0,80,0,450,0,100,0,0);
	drive.move(right, 175, 2, 60);
    findTri(tri, 24, 60);
    drive.setPID(1);
	drive.move(forward, tri.hyp + 12, 1, 100);
    drive.setPID(2);
    liftClampUp();
    pros::delay(200);

    liftPID.setTarget(3);
    drive.move(backward, 11, 1, 100);

    drive.setStandStill(DEFAULT_STANDSTILL);

    ///////////////////////////////////////////////POST 180///////////////////////////////////////////////

    //Turn to original rotation and drive backwards, then turn slightly and pick up the red
    drive.move(left, imuTarget(0), 1, 100);

    //Set turnVal to 60 for trig
    turnVal = 58;
    findTri(tri, 27, 90+turnVal);
    drive.setPID(1);
    liftPID.setTarget(0);
    drive.move(backward, tri.hyp, 2, 100);
    drive.setPID(2);
    drive.move(right, imuTarget(turnVal), 1, 70);

    //Drive forward and clamp the mogo under the platform
    drive.setPID(2);
    liftClampPassive();
    drive.move(forward, 3 + tri.b, 2, 70);
    liftClampDown();
    

    //////////////////////////////////////////GET LAST TWO REDS//////////////////////////////////////////

    //Legacy swerve method
    drive.setPID(4);
    setConveyor(0);
    drive.hardStop(backward, 3, 8, 1, 100);
	drive.swerve(backwardRight, 103, imuTarget(180), 3.3, 60, 85);


    moveDriveTrain(-8000, .2);
	backClampDown();
    moveDriveTrain(-6000, .2);


    //Turn towards the red platform (using turnVal), lift the goal and drive,
    turnVal = 245;
    liftPID.setTarget(3);
    drive.setPID(2);
    findTri(tri, 4, turnVal+90);
	drive.move(forward, tri.hyp, 1, 100);
	drive.setTemporaryPID(0,150,0,500,0,100,0,0);
	drive.move(right, imuTarget(turnVal), 1.5, 100);
    drive.setPID(1);
    setConveyor(200);
    drive.setSlew(30);
    drive.addErrorFunc(90, setSlewZero);
    drive.swerve(forward, 98 - tri.b, imuTarget(turnVal), 3, 100, 100);
    liftClampUp();
    pros::delay(200);

    ////////////////////////////////////////////SECOND 180////////////////////////////////////////////

    //Drive backwards and drop the back mogo
    drive.setStandStill(3);
    drive.setPID(2);
    drive.addErrorFunc(12, setLiftTargetZero);
	drive.move(backward, 18, 1, 100);
	backClampUp();
    pros::delay(100);
	drive.move(forward, 10, 1, 100);

    //Turn 180 degrees and pick the mogo up
	drive.setPID(1);
	drive.move(left, 180, 2, 100);
	drive.setPID(2);
    drive.addErrorFunc(4, liftClampPassive);
	drive.move(forward, 12, 1, 100);

    //Turn and drop the mogo on the platform
	liftPID.setTarget(3);
    drive.setTemporaryPID(0,80,0,450,0,100,0,0);
    turnVal = 250;
    setConveyor(0);
	drive.move(right, imuTarget(turnVal), 2, 60);
    findTri(tri, 23, turnVal);
    drive.setPID(1);
	drive.move(forward, tri.hyp + 13, 1, 100);
    drive.setPID(2);
    liftClampUp();
    pros::delay(200);

    liftPID.setTarget(3);
    drive.move(backward, 9, 1, 100);

    drive.setStandStill(DEFAULT_STANDSTILL);

    ////////////////////////////////////////////PUSH HIGH YELLOW////////////////////////////////////////////

    //Turn to be able to push the yellow to the blue side
    liftPID.setTarget(0);
    drive.setPID(1);
    drive.move(left, imuTarget(90), 1, 100);

    //Use vision swerving to correctly allign with the middle yellow then push it across the line normally
    drive.setPID(5);
    drive.visionSwerve(YELLOW_SIG_INDEX, VISION_CLOSEST, 30, 0.5, 100, 100);
    drive.setPID(1);
    drive.move(forward, 36, 4, 70);

    //Back away from the mogo (Not necessary | Simply to ensure there is no confusion if the goal counts)
    drive.move(backward, 12, 1, 100);

    //Remove tasks used during the run and clear the vector
    liftControl.remove();
    runOnError.remove();
    onErrorVector.clear();
}

void empty(){}  
