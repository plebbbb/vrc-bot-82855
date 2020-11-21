#include "BruhLibrary/global.hpp"
void initialize() {
  lcd::initialize();
  inertial.reset();
  while(inertial.is_calibrating()){
    delay(5);
  }
  delay(100);
//  autonselection();
}
//void disabled() {}
//void competition_initialize() {}
