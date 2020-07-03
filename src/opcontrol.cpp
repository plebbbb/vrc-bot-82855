#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
double ang = 0;
void opcontrol(){
  while(true){
    base.vectormove(
      ctrl.get_analog(ANALOG_LEFT_X), //left-right translation
      ctrl.get_analog(ANALOG_LEFT_Y), //forwards/back translation
      ctrl.get_analog(ANALOG_RIGHT_X), //rotation
      //10
     (speedmultiplier/127)*determinebiggest( //div by speedmultiplier/127 to scale joystick values to percentage values
      // this entire thing below is absolutely disgusting but it works
      //and also we are gonna keep with fabs cuz vectormove takes doubles
      fabs((double)ctrl.get_analog(ANALOG_RIGHT_X)),
      determinebiggest(
        fabs((double)ctrl.get_analog(ANALOG_LEFT_X)),
        fabs((double)ctrl.get_analog(ANALOG_LEFT_Y))
      )
      )
    );
    //base.vectormove(0,0,100,20);
    //base.vectormove(100,100,100,20);
    //ang+=0.01;
    //base.vectormove(cos(ang),sin(ang),0,20);
    lcd::print(0, "TR motor: %f",base.vals[0]);
    lcd::print(1, "BR motor: %f",base.vals[1]);
    lcd::print(2, "BL motor: %f",base.vals[2]);
    lcd::print(3, "TL motor: %f",base.vals[3]);
    lcd::print(5, "Rot Ratio: %f",base.rotationalratio);
    delay(10);
  };
}
