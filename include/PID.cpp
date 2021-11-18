#pragma once
#include "auton_functions.cpp"
#include <vector>
#define DEFAULT_STANDSTILL NAN
#define DISTANCE_STANDSTILL_THRESH 2
#define TURN_STANDSTILL_THRESH 0.02

#define YELLOW_SIG_INDEX 1
#define VISION_CLOSEST 0


//Direction enumeration
enum PID_direction{
  forward,
  backward,
  left,
  right,
  shortest,
  forwardRight,
  forwardLeft,
  forwardShortest,
  backwardRight,
  backwardLeft,
  backwardShortest
};


class Drive{
private:
  //Initialize PID Values
  const float pidConstants[5][8] = {
  /*{kP,kPt,kI,kIt,kD,kDt,kPd,kPs}*/
    {12,123,10,145,15,150,100,0}, //General PID
    {25,225,15,25,70,120,100,0}, //Turns under 45 degrees and short linear movements
    {12,100,10,400,15,150,100,0}, //Mogos loaded
    {50,123,10,145,15,150,100,0}, //Swerve
    {12,30,10,20,15,30,0,0} //Vision Turns and Swerve
  };

  float kP;
  float kP_t;
  float kI;
  float kI_t;
  float kD;
  float kD_t;
  float kP_d;
  float kP_s;

  float slew;
  float error;
  float standStillInput;

public:
  //Functions run when Drive is initialized
  Drive(){
    setPID(1);
    setSlew(0);
    setStandStill(DEFAULT_STANDSTILL);
  }

  ~Drive(){}

  //Public values to use for onError values
  bool isNewPID = false;

  //Setters
  void setPID(uint8_t n);
  void setTemporaryPID(float kp,float kp_t,float ki,float ki_t,float kd,float kd_t,float kp_d,float kp_s);
  void setSlew(float n);
  void setStandStill(float standStill);
  void addErrorFunc(float onError, void input());

  //Getters
  float getError();
  
  //Movement Functions
  void move(PID_direction dir, float target, float timeout, float maxVelocity);
  void swerve(PID_direction dir, float target, float target_a, float timeout, float maxVel, float maxVel_a);
  void hardStop(PID_direction dir, float targetCutOff, float target, float timeOut, float maxVelocity);
  void visionSwerve(uint8_t signature,uint8_t size,float target,float timeout,float maxVel,float maxVel_a);

};Drive drive;


//Set PID constants
void Drive::setPID(uint8_t n){
  kP = pidConstants[n-1][0];
  kP_t = pidConstants[n-1][1];
  kI = pidConstants[n-1][2];
  kI_t = pidConstants[n-1][3];
  kD = pidConstants[n-1][4];
  kD_t = pidConstants[n-1][5];
  kP_d = pidConstants[n-1][6];
  kP_s = pidConstants[n-1][7];
}

void Drive::setTemporaryPID(float kp,float kp_t,float ki,float ki_t,float kd,float kd_t,float kp_d,float kp_s){
  kP = kp;
  kP_t = kp_t;
  kI = ki; 
  kI_t = ki_t;
  kD = kd;
  kD_t = kd_t;
  kP_d = kp_d;
  kP_s = kp_s;
}

void Drive::setSlew(float n){
  slew = n*2;
}

void Drive::setStandStill(float standStill){
  standStillInput = standStill;
}

float Drive::getError(){
  return error;
}

//Basic PID movement function
void Drive::move(PID_direction dir, float target, float timeOut, float maxVelocity){
  //Error values//
  float lastError;
  float errorDrift;
  float errorSlop;
  const float initialAngle = imu.get_rotation() + 360;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  //Calc var declarations//
  float &proportion = error;
  float integral;
  float derivative;
  float proportion_drift;
  float proportion_slop;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_t = 4;
  //Motor output var declarations//
  const float maxVolt = percentToVoltage(maxVelocity);
  float finalVolt;
  //Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backward || dir == right)?(-1):(1);
  //Timeout var declaration//
  uint8_t standStillCount = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  const uint8_t standStillTolerance_t = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  if(dir == forward || dir == backward){
    while(pros::millis() < endTime && !standStill){
      error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
      if(fabs(error) < integralActive){integral = error;}
      else{integral = 0;}
      derivative = error - lastError;
      if(fabs(lastError - error) <= DISTANCE_STANDSTILL_THRESH){
        standStillCount++;
        if(standStillCount > standStillTolerance){standStill = true;}
      }
      else{standStillCount = 0;}
      master.print(2, 0, "%.2f                   ", tickToInch(error));
      lastError = error;
      if(error == 0){derivative = 0;}
      finalVolt = kP*proportion + kI*integral + kD*derivative;
      finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      if(slew){
        int avgVelocity = actualVelocityAll();
        float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
        if(fabs(tempDelta) > slew){
          float velDelta = slew;
          if(tempDelta < 0){velDelta *= -1;}
          finalVolt = velocityToVoltage(avgVelocity + velDelta);
          finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
        }
      }      
      errorDrift = imu.get_rotation() + 360 - initialAngle;
      proportion_drift = errorDrift * kP_d;
      //Set Drivetrain
      moveRightDriveTrain((reverseVal*finalVolt)-proportion_drift);
      moveLeftDriveTrain((reverseVal*finalVolt)+proportion_drift);
      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Stop the robot for 20 milliseconds
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    //Tell the onError task that the PID is over
    isNewPID = false;
    //Exit the function
    return;
  }
  else if(dir == right || dir == left){
    //Begin the PID loop
    while((pros::millis() < endTime && !standStill)){
      error = target - fabs(imu.get_rotation() + 360 - initialAngle);
      if(fabs(error) < integralActive_t){integral = error;}
      else{integral = 0;}
      derivative = error - lastError;
      if(fabs(lastError - error) <= TURN_STANDSTILL_THRESH){
        standStillCount++;
        if(standStillCount > standStillTolerance_t){standStill = true;}
      }
      else{standStillCount = 0;}
      lastError = error;
      if(error == 0){derivative = 0;}
      finalVolt = kP_t*proportion + kI_t*integral + kD_t*derivative;
      finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      master.print(2,0, "Error: %.2f", error);
      if(slew){
        int avgVelocity = actualVelocityLeft() - actualVelocityRight();
        float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
        if(fabs(tempDelta) > slew){
          float velDelta = slew;
          if(tempDelta < 0){velDelta *= -1;}
          finalVolt = velocityToVoltage(avgVelocity + velDelta);
          finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
        }
      }
      errorSlop = initialMotorAvg - motorAvgAll();
      proportion_slop = errorSlop * kP_s;
      //Set drivetrain
      moveRightDriveTrain(reverseVal*(-finalVolt)+proportion_slop);
      moveLeftDriveTrain((reverseVal*finalVolt)+proportion_slop);
      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Stop the robot for 20 millisecond
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    //Tell the onError task that the PID is over
    isNewPID = false;
    //Exit the function
    return;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drive::swerve(PID_direction dir, float target, float target_a, float timeOut, float maxVel, float maxVel_a){
  float finalValueLeft;
  float finalValueRight;
  float error_a;
  float lastError;
  float lastError_a;
  const float initialAngle = imu.get_rotation() + 360;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  //Calc var declarations//
  float &proportion = error;
  float &proportion_a = error_a;
  float integral;
  float integral_a;
  float derivative;
  float derivative_a;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_a = 4;
  //Motor output var declarations//
  const float maxVolt = percentToVoltage(maxVel);
  const float maxVolt_a = percentToVoltage(maxVel_a);
  float finalVolt;
  //Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight)?(-1):(1);
  const int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight)?(-1):(1);
  //Standstill variables//
  uint8_t standStillCount = 0;
  uint8_t standStillCount_a = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  const uint8_t standStillTolerance_a = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  bool standStill_a = false;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  //End time variable declaration//
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  while(pros::millis() < endTime && !(standStill && standStill_a)){
    /////////////////////////////////////////////////////////////////////////// DRIVE
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
    if(fabs(error) < integralActive){integral = error;}
    else{integral = 0;}
    
    derivative = error - lastError;

    //MAKE STANDSTILL WORK SOMEHOW
    if(fabs(lastError - error) <= DISTANCE_STANDSTILL_THRESH){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}

    lastError = error;
    if(error == 0){derivative = 0;}

    finalVolt = kP*proportion + kI*integral + kD*derivative;
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);

    if(slew){
      int avgVelocity = actualVelocityAll();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = velocityToVoltage(avgVelocity + velDelta);
        finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      }
    }      
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
    
    finalValueRight = reverseVal*finalVolt;
    finalValueLeft = reverseVal*finalVolt;
    master.print(2,0,"%.2f ,%.2f                  ", tickToInch(error), error_a);
    /////////////////////////////////////////////////////////////////////////// TURN
    error_a = target_a - fabs(imu.get_rotation() + 360 - initialAngle);
    
    if(fabs(error_a) < integralActive_a){integral_a = error_a;}
    else{integral_a = 0;}
    derivative_a = error_a - lastError_a;

    if(fabs(lastError_a - error_a) <= TURN_STANDSTILL_THRESH){
      standStillCount_a++;
      if(standStillCount_a > standStillTolerance_a){standStill_a = true;}
    }
    else{standStillCount_a = 0;}

    lastError_a = error_a;
    if(error_a == 0){derivative_a = 0;}

    finalVolt = kP_t*proportion_a + kI_t*integral_a + kD_t*derivative_a;
    finalVolt = mapToRange(finalVolt, maxVolt_a, -maxVolt_a);

    if(slew){
      int avgVelocity = actualVelocityLeft() - actualVelocityRight();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = velocityToVoltage(avgVelocity + velDelta);
        finalVolt = mapToRange(finalVolt, maxVolt_a, -maxVolt_a);//maybe get rid of this 
      }
    }
    //Drivetrain turn portion
    finalValueRight += reverseVal_a*(-finalVolt);
    finalValueLeft += reverseVal_a*(finalVolt);
    //Set drivetrain
    moveRightDriveTrain(finalValueRight);
    moveLeftDriveTrain(finalValueLeft);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Stop the robot for 20 milliseconds
  setBrakeMode(MOTOR_BRAKE_HOLD);
  moveDriveVoltage(0);
  pros::delay(20);
  setBrakeMode(MOTOR_BRAKE_COAST);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drive::hardStop(PID_direction dir, float targetCutOff, float target, float timeOut, float maxVelocity){
  float progress;
  float errorDrift;
  float lastError;
  const float initialAngle = imu.get_rotation() + 360;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  const float tickTargetCutOff = inchToTick(targetCutOff);
  //Calc var declarations//
  float &proportion = error;
  float proportion_drift;
  float derivative;
  //Motor output var declarations//
  const float maxVolt = percentToVoltage(maxVelocity);
  float finalVolt;
  float velDelta;
  float tempDelta;
  //Forward Backward movement multipliers
  const int8_t reverseVal = (dir == backward || dir == right)?(-1):(1);
  //Tell the onError task that a new PID has begun
  isNewPID = true;
  //Establish a cutoff in case something goes wrong
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID
  while(tickTargetCutOff > fabs(motorAvgAll()-initialMotorAvg) && (pros::millis() < endTime)){
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);

    derivative = error - lastError;
    lastError = error;

    if(error == 0){derivative = 0;}
    finalVolt = kP*proportion + kD*derivative;

    if(slew){
      int avgVelocity = actualVelocityAll();
      tempDelta = finalVolt - avgVelocity;
      if(fabs(tempDelta) > slew){
        velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
      }
      finalVolt = avgVelocity + velDelta;
    }

    master.print(2,0, "Error: %.2f", error);

    //Set finalVolt to range
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);

    errorDrift = imu.get_rotation() + 360 - initialAngle;
    proportion_drift = errorDrift * kP_d;
    //Set Drivetrain
    moveRightDriveTrain((reverseVal*finalVolt)-proportion_drift);
    moveLeftDriveTrain((reverseVal*finalVolt)+proportion_drift);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Set voltage to 0 in case this is the last function called in an autonomous
  moveDriveVoltage(0);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drive::visionSwerve(uint8_t signature,uint8_t size,float target,float timeOut,float maxVel,float maxVel_a){
  float finalValueLeft;
  float finalValueRight;
  float error_a;
  float lastError;
  float lastError_a;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  const float error_aCutoff = inchToTick(7);
  //Calc var declarations//
  float &proportion = error;
  float &proportion_a = error_a;
  float integral;
  float integral_a;
  float derivative;
  float derivative_a;
  //integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_a = 4;
  //Motor output var declarations//
  const float maxVolt = percentToVoltage(maxVel);
  const float maxVolt_a = percentToVoltage(maxVel_a);
  float finalVolt;
  //Standstill variables//
  uint8_t standStillCount = 0;
  uint8_t standStillCount_a = 0;
  const uint8_t standStillTolerance = ((standStillInput==standStillInput)?(standStillInput):(10));
  const uint8_t standStillTolerance_a = ((standStillInput==standStillInput)?(standStillInput):(10));
  bool standStill = false;
  bool standStill_a = false;
  //Tell the onError task that a new PID has begun//
  isNewPID = true;
  //End time variable declaration//
  const uint32_t endTime = pros::millis() + timeOut*1000;
  //Begin PID//
  while(pros::millis() < endTime && !(standStill && standStill_a)){
    /////////////////////////////////////////////////////////////////////////// DRIVE
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);
    if(fabs(error) < integralActive){integral = error;}
    else{integral = 0;}
    
    derivative = error - lastError;

    //MAKE STANDSTILL WORK SOMEHOW
    if(fabs(lastError - error) <= DISTANCE_STANDSTILL_THRESH){
      standStillCount++;
      if(standStillCount > standStillTolerance){standStill = true;}
    }
    else{standStillCount = 0;}

    lastError = error;
    if(error == 0){derivative = 0;}

    finalVolt = kP*proportion + kI*integral + kD*derivative;
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);

    if(slew){
      int avgVelocity = actualVelocityAll();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = velocityToVoltage(avgVelocity + velDelta);
        finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
      }
    }      
    finalVolt = mapToRange(finalVolt, maxVolt, -maxVolt);
    
    finalValueRight = finalVolt;
    finalValueLeft = finalVolt;
    master.print(2,0,"%.2f ,%.2f                  ", tickToInch(error), error_a);
    /////////////////////////////////////////////////////////////////////////// TURN
    pros::vision_object_s_t visionClosestBySig = vision.get_by_sig(0, 1);
    error_a = -(visionClosestBySig.x_middle_coord - 20) * (error > error_aCutoff);
    
    if(fabs(error_a) < integralActive_a){integral_a = error_a;}
    else{integral_a = 0;}
    derivative_a = error_a - lastError_a;

    if(fabs(lastError_a - error_a) <= TURN_STANDSTILL_THRESH){
      standStillCount_a++;
      if(standStillCount_a > standStillTolerance_a){standStill_a = true;}
    }
    else{standStillCount_a = 0;}

    lastError_a = error_a;
    if(error_a == 0){derivative_a = 0;}

    finalVolt = kP_t*proportion_a + kI_t*integral_a + kD_t*derivative_a;
    finalVolt = mapToRange(finalVolt, maxVolt_a, -maxVolt_a);

    if(slew){
      int avgVelocity = actualVelocityLeft() - actualVelocityRight();
      float tempDelta = voltageToVelocity(finalVolt) - avgVelocity;
      if(fabs(tempDelta) > slew){
        float velDelta = slew;
        if(tempDelta < 0){velDelta *= -1;}
        finalVolt = velocityToVoltage(avgVelocity + velDelta);
        finalVolt = mapToRange(finalVolt, maxVolt_a, -maxVolt_a);//maybe get rid of this 
      }
    }
    //Drivetrain turn portion
    finalValueRight += (-finalVolt);
    finalValueLeft += (finalVolt);
    //Set drivetrain
    moveRightDriveTrain(finalValueRight);
    moveLeftDriveTrain(finalValueLeft);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Stop the robot for 20 milliseconds
  setBrakeMode(MOTOR_BRAKE_HOLD);
  moveDriveVoltage(0);
  pros::delay(20);
  setBrakeMode(MOTOR_BRAKE_COAST);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return;
}
