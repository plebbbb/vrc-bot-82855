#include "BruhLibrary/global.hpp"

/*
A guide to the current autonomous approach
  1. Commands are to be updated via adjusting positional variables
  2. There is to be no direct interfacing with class functions

The idea is, we use positional variables, which the wrappers are pointed to
There really isn't an advantage to doing stuff like this, I just thought it looked
cool. In the 65% chance this approach is risky, we can switch to a function based
system

*/
/*
std::vector<linearmotion> twentyseven{
  linearmotion(
    0, 72, M_PI/2, // x, y, angle
    new intakecommandset(
      new std::vector<std::vector<double>>{
      {100,100,99,100,300}
    }, &intakes)
  ),

  linearmotion(
    0, 72, M_PI/2,
    new intakecommandset(new std::vector<std::vector<double>> {
      {100, 0, 0, 100, 500}
    }, &intakes)
  )
};
*/
std::vector<linearmotion> linemoves = {
  linearmotion(0, 24, M_PI/2),
  linearmotion(0.00 , 24 , 0.00),
  linearmotion(0,0,0.00)
};

/*
std::vector<linearmotion> circularpath = {
  linearmotion(0, 24, M_PI),
  linearmotion(36, 24, M_PI*3/2),
  linearmotion(72, 24, 0.0),
  linearmotion(72, 60, 0.0),
  linearmotion(36, 96, M_PI/2),
  linearmotion(0, 96, 0.0),
  linearmotion(0, 24, M_PI),
  linearmotion(0, 24, M_PI),
  linearmotion(36, 24, M_PI*3/2),
  linearmotion(72, 24, 0.0),
  linearmotion(72, 60, 0.0),
  linearmotion(36, 96, M_PI/2),
  linearmotion(0, 96, 0.0),
  linearmotion(0, 24, M_PI),
  linearmotion(0, 24, M_PI),
  linearmotion(36, 24, M_PI*3/2),
  linearmotion(72, 24, 0.0),
  linearmotion(72, 60, 0.0),
  linearmotion(36, 96, M_PI/2),
  linearmotion(0, 96, 0.0),
  linearmotion(0, 24, M_PI),
  linearmotion(0, 24, M_PI),
  linearmotion(36, 24, M_PI*3/2),
  linearmotion(72, 24, 0.0),
  linearmotion(72, 60, 0.0),
  linearmotion(36, 96, M_PI/2),
  linearmotion(0, 96, 0.0),
  linearmotion(0, 24, M_PI),
  linearmotion(0, 24, M_PI),
  linearmotion(36, 24, M_PI*3/2),
  linearmotion(72, 24, 0.0),
  linearmotion(72, 60, 0.0),
  linearmotion(36, 96, M_PI/2),
  linearmotion(0, 96, 0.0),
  linearmotion(0, 24, M_PI)
};

std::uint32_t oldtime = 0;
//note: old auton config info here has been depreciated. New auton commands in global.cpp, may decide to move it over.
//also, the old line testing code has been removed. See old commits for it, like pre october or something
/********************************************************************************/
void autonomous(){
  inertial.reset();
  while(inertial.is_calibrating()){
    delay(5);
  }
  int arr = 0;
  linemoves[arr].set_tgt();
  while(true){
    double newang = fmod(fmod(degtorad(-inertial.get_heading()),M_PI*2)+ M_PI/2,M_PI*2);
    odo.posupdvIMU(newang-angleG);
//    lcd::clear();
    odometrycontrollerdebug();
    if(/*linemoves[arr].updatesystems() &&*/ mover.update()){
      arr++;
      if(linemoves.size() == arr) arr--;
      linemoves[arr].set_tgt();
    }
    std::uint32_t time = pros::millis();
    pros::Task::delay_until(&time,10);
    //delay(10);
  }
/*  motionpaths[arr].computepath();
  while(true){
    odo.posupdv2();
    odometrycontrollerdebug();
    if (mover.updateMP()){
      if (motionpaths[arr].computepath() && arr < motionpaths.size()-1) arr += 1;
    }
    delay(10); //refresh clock
  }
  */

}
