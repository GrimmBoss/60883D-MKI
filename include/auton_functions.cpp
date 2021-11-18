#pragma once
#include "include.cpp"
#define MIN_VOLTAGE -12000
#define MAX_VOLTAGE 12000
#define PI 3.14159


void setBrakeMode(pros::motor_brake_mode_e brakeMode){
  frontright.set_brake_mode(brakeMode);
  midright.set_brake_mode(brakeMode);
  backright.set_brake_mode(brakeMode);
  frontleft.set_brake_mode(brakeMode);
  midleft.set_brake_mode(brakeMode);
  backleft.set_brake_mode(brakeMode);
}

void moveLeftDriveTrain(int voltage){
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
}

void moveRightDriveTrain(int voltage){
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveVoltage(int voltage){
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}

int motorAvgLeft(){
  return(backleft.get_position()+midleft.get_position()+frontleft.get_position())/3;
}

int motorAvgRight(){
  return(frontright.get_position()+midright.get_position()+backright.get_position())/3;
}

int motorAvgAll(){
  return(motorAvgLeft()+motorAvgRight())/2;
}

float actualVelocityLeft(){
  return(frontleft.get_actual_velocity() + midleft.get_actual_velocity() + backleft.get_actual_velocity())/3;
}

float actualVelocityRight(){
  return(frontright.get_actual_velocity() + midright.get_actual_velocity() + backright.get_actual_velocity())/3;
}

//returns a float between -200 and 200
float actualVelocityAll(){
  return(actualVelocityLeft() + actualVelocityRight())/2;
}

template <typename T>
T mapToRange(T num, T maxVal, T minVal){
  num = MIN(num, maxVal);
  num = MAX(num, minVal);
  return(num);
}

float radToDeg(float radian){
  return(radian * (180/PI));
}

float degToRad(float degree){
  return(degree * (PI/180));
}


//1800 ticks/rev with 100 rpm cartridge
//900 ticks/rev with 200 rpm cartridge
//300 ticks/rev with 600 rpm cartridge
//642.86 ticks for one full wheel rotation (900 * (60/84) or (5/7))
//Circumference of 4" omni = 12.96 (4.125*pi)
//642.86 ticks / 12.96 in = 49.60 ticks per inch
int inchToTick(float inch){
  return(inch * 49.6);
}

float tickToInch(int tick){
  return(tick / 49.6);
}

float percentToVoltage(float percent){
  return(percent * 120);
}

//(voltage / 120)/2
float voltageToVelocity(float voltage){
  return(voltage / 60);
}

int velocityToVoltage(float percent){
  return(percent * 60);
}

//Returns the shortest possible angle between target and current angle
float imuTarget(float target){
  return(MIN(360 - fabs((fabs(target) - imu.get_heading())), fabs(target - imu.get_heading())));
}

//Stupid ugly function with nested if loops to get the closest value to the passed through value from an array
int getClosestIndicator(int liftRot){
  //Make sure this is the same as the lift PID constants
  liftRot = float(liftRot)/100;
  if (liftRot - liftTargets[0] >= liftTargets[1] - liftRot){
    if(liftRot - liftTargets[1] >= liftTargets[2] - liftRot){
      if(liftRot - liftTargets[2] >= liftTargets[3] - liftRot){
        return 3;
      }
      return 2;
    }
    return 1;
  }
  return 0;
}

void deploy(){
  conveyor.move_voltage(-4000);
  pros::delay(80);
  conveyor.move_velocity(0);
}

void platformBackClamp(){
  moveDriveTrain(-6000, .3);
  backClamp.set_value(true);
  moveDriveTrain(3000, .1);
}

//Set speed between -200 and 200
void setConveyor(int speed){
  conveyor.move_velocity(speed * 2);
}

struct Triangle {
public:
  float a;
  float b;
  float hyp;
  float alpha;
  float beta;
};

/**
 * Does trigonometry to assign values to attributes of a given Triangle object
 *
 * \param obj
 *        Reference to the Triangle object who's attributes will be set
 * \param a
 *        Leg A of the triangle
 * \param reference
 *        Angle to subtract from the current IMU heading. 
 *        Will set hyp equal to A if reference and IMU heading are the same.
 *        
 */
void findTri(Triangle& obj, float a, float reference){
  obj.beta = degToRad(imu.get_heading() - reference);
  obj.alpha = 90 - obj.beta;
  obj.a = a;
  obj.b =  a * tan(obj.beta);
  obj.hyp = sqrt(a*a + obj.b*obj.b);
}