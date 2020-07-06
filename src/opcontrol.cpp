#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
double ang = M_PI/2;

//*******************************************************************************//
//Control scheme configuration
//array format: left-right, forwards-back, clockwise-counterclockwise
controller_analog_e_t controlscheme[]{
  ANALOG_LEFT_X,
  ANALOG_LEFT_Y,
  ANALOG_RIGHT_X
};

//Control scheme featureset
//array format: enable relative mode, enable angle hold
bool configoptions[]{
  false,
  true
};

//*******************************************************************************//
//mover configuration



//*******************************************************************************//
//The actual code
void opcontrol(){
  opcontrolcontroller useonlyinopcontrol(base,controlscheme,bPID[2],configoptions);
  xyaT[0] = 20;
  xyaT[1] = 10;
  xyaT[2] = 3.14;
  while(true){
    odo.posupdv2();
    //useonlyinopcontrol.ssc->vectormove(10, 10, 0, 10);
    //lcd::print(1,"%f",useonlyinopcontrol.ssc->MAP[0].cosV);
    //useonlyinopcontrol.relativemove(ctrl.get_analog(ANALOG_RIGHT_X));
    useonlyinopcontrol.move();
    if (ctrl.get_digital_new_press(DIGITAL_B)) configoptions[0] = !configoptions[0];
    delay(10);
  };
//below: test for autonomous

}
