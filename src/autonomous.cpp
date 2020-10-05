#include "main.h"
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

//********************************************************************************//
/*auton procedures:
procedure documentation:
  x coord, y coord, heading angle, final orientation angle, motorF values...
*/
std::vector<std::vector<double>> moveinst[] = {
  //moveinst[0] - A test instance for solely base movement
  {
    {0,0,M_PI/2,M_PI/2},
    {10,10,0,M_PI/2},
    {20,20,M_PI/2,M_PI/2}
  }
};

std::vector<motion> processedpaths = {
  //motion(moveinst[0].data(),moveinst[0].data()->size(),0,1,curvesets[0])
  //motion(moveinst[0].data(),3,0,1,curvesets[0])
  //TBD: does moveinst[0].data work? and does the casting to double** work?
};

//postionsetTEST: Benchmark test to ensure the functionality of coordcontroller in direct line mode
/*procedure documentation:
  x coord, y coord, angle target... TBA
*/
double positionsetTEST[][3] = {
  {0,0,M_PI/2},
  {30,15,M_PI},
  {10,10,(3*M_PI)/2},
  {20,25,(M_PI)/5},
  {0,30,M_PI*2},
  {0,0,M_PI/2}
};

//********************************************************************************//
void autonomous(){
  int cycle = 0;
  xyaT[0] = positionsetTEST[0][0];
  xyaT[1] = positionsetTEST[0][1];
  xyaT[2] = positionsetTEST[0][2];
  coordcontroller mover(base,bPID); //TBD: fix this
  double perc = 0;
  int arr = 0;
  while(true){
    odo.posupdv2();
    if (mover.updateMP()){
      if (motionpaths[arr].computepath() && arr < motionpaths.size()) arr += 1;
    }
  }
}
