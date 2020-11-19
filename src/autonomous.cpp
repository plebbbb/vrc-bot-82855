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

std::vector<linearmotion> linemoves{
  linearmotion(
    0, //x
    0,  //y
    new orientationscheme(
        *new std::vector<std::vector<double>>{
          {M_PI/2,50,100}
        }
      ),
    new intakecommandset(
      new std::vector<std::vector<double>>{
      {100,100,24,100,100},
      {0,100,0,100,100}
    },
    &intakes
    )
  ),
  /*linearmotion(b
    15,
    15,
    new orientationscheme(
        *new std::vector<std::vector<double>>{
          {M_PI*3/4,0,25},
        }
      )
  )*/
};
bool flag = false;
void globalclock(){
  flag = !flag;
  delay(5); //10sec to invert itself back to right state\

}
std::uint32_t oldtime = 0;
//note: old auton config info here has been depreciated. New auton commands in global.cpp, may decide to move it over.
//also, the old line testing code has been removed. See old commits for it, like pre october or something
/********************************************************************************/
void autonomous(){
  int arr = 0;
  linemoves[arr].set_tgt();
  while(true){
    odo.posupdv2();
  //  odometrycontrollerdebug();
    if(linemoves[arr].updatesystems() && mover.update()){
      arr++;
      if(linemoves.size() == arr) arr--;
      linemoves[arr].set_tgt();
    }
    lcd::clear();
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
