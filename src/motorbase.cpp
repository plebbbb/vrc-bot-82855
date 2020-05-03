#include "main.h"
#include "motorbase.hpp"
//below is motor array for a kiwi motor config
motorw kiwimotors[] = {
  motorw(1,true,M_PI_2/3,2.5), //right motor
  motorw(2,true,M_PI_4/3,2.5), //left motor
  motorw(3,true,0,2.5) //Pure translational Motor
};
basecontroller base(kiwimotors);
