#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
void initialize() {
  lcd::initialize();
  curvesets[0].a = &curves[0];
  curvesets[0].b = &curves[1];
  bPID[0].Scurve = &curvesets[0];
  //coordcontroller mover(base,bPID,xyaT);
  delay(100);
  //autonselection(); //calls auton selection method
}
//void disabled() {}
//void competition_initialize() {}
