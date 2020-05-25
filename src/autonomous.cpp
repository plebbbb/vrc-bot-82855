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
  x coord, y coord, angle target... TBA
*/

//postionsetTEST: Benchmark test to ensure the functionality of the motors
double positionsetTEST[][3] = {
  {100,100,1},
  {200,100,2},
  {50,50,2*M_PI},
  {-25,200,(5*M_PI)/3}
};

//********************************************************************************//
void autonomous(){
  int cycle = 0;
  while(true){
    odo.updateposition();
    //the idea for this if statement is that it calls all updates
    //and only passes once everything is done, before updating the variables
    if (mover.update()/* && (motorf object).PID_UPDATE_CYCLE ....*/){
      /*TBD: Make the movement array into a 3d array,
        have the third dimension be for each different auton
        and the */
      cycle++;
      //is the for loop cheese? theres the something:something way but
      //idk how that works, nor if its even in C++
      for (int i = 0; i > 3; i++){
        xyaT[i] = positionsetTEST[cycle][i];
      }
      /*while the goal was to not use functions, chances are we are gonna
      have to for motorf because I really dont wanna rewrite those constructors*/
      //suggested code for doing so//
      /*
      (motorf object).PID_MOVE_TARGET(positionsetTEST[cycle][specific index]);
      */
      delay(10);
    }
  }
}
