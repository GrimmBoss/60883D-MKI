#pragma once
#include "auton_functions.cpp"

enum liftClampState{
    passiveExtend = 1,
    activeExtend = 2,
    activeRetract = 3
};

class PneumaticControl{
public:
    //Setters
    void setBackClamp(bool state);
    void toggleBackClamp();

    void setliftClamp(liftClampState state);
    void toggleLiftClamp();
    void lockLiftClamp();

    bool backPistonToggle = LOW;
    bool liftSol1Toggle = LOW;
    bool liftSol2Toggle = HIGH;


}; PneumaticControl pneumatics;

void PneumaticControl::toggleBackClamp(){
    backPistonToggle =! backPistonToggle;
	backClamp.set_value(backPistonToggle);
}

void PneumaticControl::setBackClamp(bool state){
    backPistonToggle = state;
    backClamp.set_value(backPistonToggle);
}

void PneumaticControl::setliftClamp(liftClampState state){
    passiveExtend;
    activeExtend;
    activeRetract;

    liftSol1Toggle = !(state%2);
    liftSol2Toggle = (state-1);
    
    liftSol1.set_value(liftSol1Toggle);
    liftSol2.set_value(liftSol2Toggle);
}

void PneumaticControl::toggleLiftClamp(){
    liftSol1Toggle = LOW;
    liftSol1.set_value(LOW);

    liftSol2Toggle =! liftSol2Toggle;
    liftSol2.set_value(liftSol2Toggle);
}

void PneumaticControl::lockLiftClamp(){
    liftSol1Toggle = HIGH;
    liftSol1.set_value(HIGH);

    liftSol2Toggle = LOW;
    liftSol2.set_value(LOW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////


class Lift{
private:
    //Initialize PID Values
    const float kP = 550;
    const float kI = 10;
    const float kD = 0;
    const float intergralActive = 2;
    const float intergralLimit = 40;

    //Variables that need to be read after each PID scope is destroyed
    float error;
    float lastError;
    float intergral;
    
public:
    Lift(){
        setTarget(getClosestIndicator(liftPot.get_angle()));
    }

    uint8_t targetIndicator;
    float target;
    void PID();
    void waitUntilTargetReached(float timeOut);
    void setTarget(int n);
    void setCustomTarget(float target);

};Lift liftPID;


void Lift::setTarget(int n){
    targetIndicator = mapToRange(n, LIFTVALSIZE-1, 0);
    target = liftTargets[targetIndicator];
}

void Lift::setCustomTarget(float customTarget){
    target = customTarget;
}

void Lift::waitUntilTargetReached(float timeOut){
    const uint32_t endTime = pros::millis() + timeOut*1000;
    while(pros::millis() < endTime && fabs(error) < 3){
        pros::delay(10);
    }
}

void Lift::PID(){
    error = target - (float(liftPot.get_angle())/100);
    float proportion = error;

    if(fabs(error) <= intergralActive){intergral += error;}
    else{intergral = 0;}
    intergral = mapToRange(intergral, intergralLimit, -intergralLimit);

    float derivative = error - lastError;
    lastError = error;
    if(error == 0){derivative = 0;}

    int finalVolt = kP*proportion + kI*intergral + kD*derivative;

    //Set finalVolt to range
    finalVolt = mapToRange(finalVolt, MAX_VOLTAGE, MIN_VOLTAGE);

    //master.print(2,0,"%.2f, %.0f                ", error, target);

    //Set final lift speeds
    lift.move_voltage(finalVolt);
}

void liftPID_fn(void *param){
    std::uint32_t startTime = pros::millis();
    while(true){
        liftPID.PID();
        pros::Task::delay_until(&startTime, 10);
    }
}