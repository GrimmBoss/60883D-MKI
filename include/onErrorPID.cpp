#include "PID.cpp"
#include "robotClasses.cpp"

struct errorFuncTuple{
   std::function<void()> func;
   float onError;
   bool called;
   errorFuncTuple(std::function<void()> func, float onError, bool called):
   func(func),onError(onError), called(called){}
};

std::vector<errorFuncTuple> onErrorVector;

void Drive::addErrorFunc(float onError, void input()){
    //TODO add easy integration with turning (although not that useful)
    onErrorVector.emplace_back(errorFuncTuple(input, inchToTick(onError), false));
}

pros::Mutex onErrorMutex;

void onError_fn(void* param){
  std::uint32_t startTime = pros::millis();
  while(true){
  onErrorMutex.take(TIMEOUT_MAX);
  if(drive.isNewPID){
    auto iter = onErrorVector.begin();
    while(iter != onErrorVector.end()){
      if (!iter->called && (iter->onError >= drive.getError())){
        iter->func();
        iter->called = true;
      }
      else{iter++;}
      }
    }
    onErrorMutex.give();
    pros::Task::delay_until(&startTime, 10);  
  }
}

//Functions called when onError is met
void backClampDown(){pneumatics.setBackClamp(HIGH);}
void backClampUp(){pneumatics.setBackClamp(LOW);}
void liftClampPassive(){pneumatics.setliftClamp(passiveExtend);}//Fix these
void liftClampDown(){pneumatics.setliftClamp(activeExtend);}
void liftClampUp(){pneumatics.setliftClamp(activeRetract);}
void setLiftTargetThree(){liftPID.setTarget(3);}
void setLiftTargetOne(){liftPID.setTarget(1);}
void setLiftTargetZero(){liftPID.setTarget(0);}
void setSlewZero(){drive.setSlew(0);}
