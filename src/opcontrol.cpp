#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
double ang = M_PI/2;
void opcontrol(){
  xyaT[0] = 20;
  xyaT[1] = 10;
  xyaT[2] = 3.14;
  while(true){
    odo.posupdv2();
  //  mover.update();
  /*  base.vectormove(
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
    double output = 0;
    if (ctrl.get_analog(ANALOG_RIGHT_X)) {
      ang = angleG;
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
        );
      }
      else{
        output = bPID[2].update(getrelrad(angleG,ang));
        base.vectormove(
            ctrl.get_analog(ANALOG_LEFT_X), //left-right translation
            ctrl.get_analog(ANALOG_LEFT_Y), //forwards/back translation
            -output, //rotation
            //10
           (speedmultiplier/127)*determinebiggest( //div by speedmultiplier/127 to scale joystick values to percentage values
            // this entire thing below is absolutely disgusting but it works
            //and also we are gonna keep with fabs cuz vectormove takes doubles
            fabs(output),
            determinebiggest(
              fabs((double)ctrl.get_analog(ANALOG_LEFT_X)),
              fabs((double)ctrl.get_analog(ANALOG_LEFT_Y))
            )
            )
          );
      }
      lcd::print(0,"PID output: %f",output);
      lcd::print(1,"Target Angle: %f", ang);
      lcd::print(2,"Current Angle: %f", angleG);
      lcd::print(3,"Difference to tgt: %f", getrelrad(angleG,ang));
    //base.vectormove(0,0,100,20);
    //base.vectormove(100,100,100,20);
    //ang+=0.01;
    //base.vectormove(cos(ang),sin(ang),0,20);
    delay(25);
  };
}
