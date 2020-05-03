#include "main.h"
using namespace pros;
/*motorw: A motor wrapping function which provides the following features:
  - Additional information
    - Motor orientation data
    - Motor wheel size with calculations
  - PID motor control
*/
struct motorw{

};
/*motorbase: A wrapper for motorw which seeks to control base movement:
  - Parametric motor configurations
  - Directional control with speed indication
  - Rotational control with PID stabilization
*/
struct basecontroller{

};
extern basecontroller base();
extern motorw motors[];
