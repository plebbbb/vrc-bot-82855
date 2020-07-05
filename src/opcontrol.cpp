#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
double ang = 0;
void opcontrol(){
  xyaT[0] = 20;
  xyaT[1] = 10;
  xyaT[2] = 3.14;
  while(true){
    odo.posupdv2();
  //  mover.update();
    base.vectormove(
      ctrl.get_analog(ANALOG_LEFT_X), //left-right translation
      ctrl.get_analog(ANALOG_LEFT_Y), //forwards/back translation
      -ctrl.get_analog(ANALOG_RIGHT_X), //rotation
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
    );/*/
    /*lcd::print(0,"Left Encoder: %f", (rottodist(degtorad((double)odo.left->get_value()),(double)STD_TWHEEL_RADIUS)));
    lcd::print(1,"Right Encoder: %f", (rottodist(degtorad((double)odo.right->get_value()),(double)STD_TWHEEL_RADIUS)));
    lcd::print(2,"Back Encoder: %f", (rottodist(degtorad((double)odo.back->get_value()),(double)STD_BTWHEEL_RADIUS)));
  /*/  lcd::print(0,"X: %f",xG);
    lcd::print(1,"Y: %f", yG);
    lcd::print(2,"Angle: %f", angleG);
    lcd::print(3,"Est Spd: %d in/s", (int)estspd);
    lcd::print(4, "Est Heading: %f", heading);
  /*  lcd::print(3,"LD: %f",odo.LD);
    lcd::print(4,"RD: %f",odo.RD);
    lcd::print(5,"HD: %f", odo.HD);
    lcd::print(6,"RA: %f", odo.relangle);
    lcd::print(7,"DS: %f", (odo.RD-odo.LD));*/

    //base.vectormove(0,0,100,20);
    //base.vectormove(100,100,100,20);
    //ang+=0.01;
    //base.vectormove(cos(ang),sin(ang),0,20);
    delay(10);
  };
}
