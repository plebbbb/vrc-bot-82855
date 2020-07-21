#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
void initialize() {
  lcd::initialize();
  /*So these shouldnt be here but basically because we are totally dependent on pointers for everything
  and also some things haven't been given a memory space or something we have to do all this stuff at runtime
  and yeah this is obviously kinda retarded so its gonna have to get fixed but also idk how to do it*/
  curvesets[0].a = &curves[0]; //tbd fix this memory issue?
  curvesets[0].b = &curves[1];
  bPID[0].Scurve = &curvesets[0];
  //coordcontroller mover(base,bPID,xyaT);
  delay(100);
  //autonselection(); //calls auton selection method
}
//void disabled() {}
//void competition_initialize() {}
