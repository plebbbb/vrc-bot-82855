#include "BruhLibrary/global.hpp"

//*******************************************************************************//
//Control scheme configuration
//array format: left-right, forwards-back, clockwise-counterclockwise
/*controller_analog_e_t controlscheme[]{
  ANALOG_LEFT_X,
  ANALOG_LEFT_Y,
  ANALOG_RIGHT_X
};

//Control scheme featureset
//array format: enable relative mode, enable angle hold
bool configoptions[]{
  false,
  true
};*/

//Above: probably fixed the problem forcing this to be here, if not, above was a working option

//*******************************************************************************//
//The actual code
//Imu asdasd(11);
void opcontrol(){
  //TBD: fix the pointers on these so they actually work - probably done see older revisions for all the old stuff needed to get it to work
  // autonomous();
  inertial.reset();
  while(inertial.is_calibrating()){
    delay(5);
  }
  int Oang = M_PI/2;
  while(true){
//    intakes.intake_velocity(200*(ctrl.get_digital(DIGITAL_L1)-ctrl.get_digital(DIGITAL_L2)),200*(ctrl.get_digital(DIGITAL_R1)-ctrl.get_digital(DIGITAL_R2)));
      angleG = getrelrad(fmod(fmod(degtorad(inertial.get_heading()),M_PI*2)+M_PI/2,M_PI*2), angleG);
  //  odo.posupdv2();
  //  lcd::clear();
    odometrycontrollerdebug();
    lcd::print(6,"%d",odo.PLV);
    lcd::print(7,"%d",odo.PBV);
    //useonlyinopcontrol.ssc->vectormove(10, 10, 0, 10);
    //useonlyinopcontrol.relativemove(ctrl.get_analog(ANALOG_RIGHT_X));
    //odometrycontrollerdebug();

    // intakes.input();
    useonlyinopcontrol.move();
    if (ctrl.get_digital_new_press(DIGITAL_B)) configoptions[0] = !configoptions[0];
    if (ctrl.get_digital_new_press(DIGITAL_A)) configoptions[1] = !configoptions[1];
    if (ctrl.get_digital_new_press(DIGITAL_Y)) configoptions[2] = !configoptions[2];
    delay(10);
  }; /*
//below: test for autonomous
  while(true){
    odo.posupdv2();
    mover.update();
    odometrycontrollerdebug();
    delay(10);
  }*/
//  autonomous();
}
