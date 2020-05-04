#include "main.h"
#include "motorbase.hpp"
using namespace pros;
void opcontrol(){
  while(true){
    base.vectormove(
      ctrl.get_analog(ANALOG_LEFT_X), //left-right translation
      ctrl.get_analog(ANALOG_LEFT_Y), //forwards/back translation
      ctrl.get_analog(ANALOG_RIGHT_X), //rotation
      speedmultiplier*determinebiggest(
      // this entire thing below is absolutely disgusting but it works
      fabs(ctrl.get_analog(ANALOG_RIGHT_X)),
      determinebiggest(fabs(ctrl.get_analog(ANALOG_RIGHT_X)),
      fabs(ctrl.get_analog(ANALOG_LEFT_Y))))
    );
    delay(10);
  };
}
