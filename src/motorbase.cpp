#include "main.h"
#include "motorbase.hpp"
// This cpp is for the declration of extern objects from motorbase.hpp

/*a tip making motorw arrays:
  it is suggested to not inverse motor direction, and instead figure out the
  correct vector direction for positive movement. This prevents potential issues
  with rotation.
*/

//motorw arrays:
//motorw format: pin, inv dir, orientation, wheel size
motorw kiwimotors[] = {
  motorw(1,true,(M_PI*2)/3,2.5), //right motor
  motorw(2,true,(M_PI*4)/3,2.5), //left motor
  motorw(3,true,0,2.5) //Pure translational Motor
};


//basecontrollers:
basecontroller base(kiwimotors);
