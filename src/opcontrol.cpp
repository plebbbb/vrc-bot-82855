#include "main.h"
#include "motorbase.hpp"
using namespace pros;
void opcontrol(){
  while(true){
    base.vectormove(
      ctrl.get_analog(ANALOG_LEFT_X), //left-right translation
      ctrl.get_analog(ANALOG_LEFT_Y), //forwards/back translation
      ctrl.get_analog(ANALOG_RIGHT_X), //rotation
      speedmultiplier*determinebiggest({
        ctrl.get_analog(ANALOG_LEFT_X),
        ctrl.get_analog(ANALOG_LEFT_Y),
        ctrl.get_analog(ANALOG_RIGHT_X)
      });
      delay(10);
  };
}
