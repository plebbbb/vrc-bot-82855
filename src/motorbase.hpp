#include "main.h"
using namespace pros;
/*ADVANCE DECLARATIONS*/

struct motorw; struct odometrycontroller; struct basecontroller;


/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 10 //offset from center line of the y axis tracking sheel
#define X_AXIS_TWHEEL_OFFSET 7.5 //not being used currently. we'd need another horz wheel for that
#define STD_WHEEL_RADIUS 1.875 //3.75in wheel
#define STD_TWHEEL_RADIUS 1.25 //2.5in wheel


/*GLOBAL VARIABLES*/

extern Controller ctrl;
extern double angleG;
extern double xG;
extern double yG;
extern double speedmultiplier;
extern odometrycontroller odo;
extern motorw basemotors[];
extern basecontroller base;


/*UTLITITY FUNCTIONS*/

//for some reason, sort() is broken so i have to diy something,
//This directly sorts the inputted member, no returns
extern void insertionsort(double arr[]);
//determinebiggest: returns biggest number, not absolute
extern double determinebiggest(double a, double b);
//isposorneg: returns 1 or -1 depending on if the value is positive or not
extern double isposorneg(double input);
//getrelrad: assumes positive radians, whereby 90deg right is 0
extern double getrelrad(double crad, double trad);
//rottodist: converts radians into distance based on the radius of rotator
extern double rottodist(double rad, double radius);


/*CLASS DECLARATIONS*/

//dualScurve: a set of 2 S curves made to aproximate what motion profiling might look like
struct dualScurve{

};

/*PID: generic PID system*/
class PID{
private:
  /*Integral mode configurations:
  0: Direct I scaling - 100% of I is added each cycle
  1: Asymptope I - I approaches the max*/
  int Imode = 0;
  /*Proportional mode configurations
  0: Raw input
  1: External S curve, percent to target based
  */
  int Pmode = 0;
  //Izerocutoff: turns I to 0 when target reached
  bool Izerocutoff = true;
  //Pk, Ik and Dk array
  double* ratios;
  //selected dualScurve
  dualScurve* Scurve;
public:
  PID(double scalers[], double modesettings[]){
    ratios = scalers;
  }
  PID(double scalers[], double modesettings[], dualScurve curve){
    ratios = scalers; Scurve = &curve;
  }
};

/*motorf: A motor wrapping function for non-base motors which provides the following:
  - PID control system
  - Internal angle scaling system
  - Built in encoder based control
  - Multiple input control schemes
    - Toggles
    - Double button hold
    - Joystick axis
  - Automated background operation based on sensor inputs
*/
struct motorf{
  ADIEncoder* linkedencoder;
  //PID_MOVE_TARGET
  void PID_MOVE_TARGET(){

  }
  //PID_MOVE_CYCLE: One increment PID update system, returns movement completion
  bool PID_MOVE_CYCLE(){

  }
};

/*motorw: A motor wrapping funion for bases which provides the following features:
  - Additional information
    - Motor orientation data
    - Motor wheel size with calculations (INCHES)
*/
struct motorw{
  Motor mot;
  double orientation;
  double cosV, sinV;
  motorw(int pin, bool dir, double o):mot(pin,dir),orientation(o){
    //precalculated values for basecontroller vector control
    cosV = cos(orientation); sinV = sin(orientation);
  }
};

/*motorbase: A wrapper for motorw which seeks to control base movement:
  - Parametric motor configurations
  - Directional control with speed controls
  - Rotational control with PID stabilization
*/
struct basecontroller{
  motorw* MAP; //sketchy pointer that points to the motorw array so we can use it later
  basecontroller(motorw m[]){MAP = m;}
  /*vectormove: a universal movement function which takes x and y inputs,
  magnitude is irrelevant in this case, use spd to determine speed*/
  void vectormove(double x, double y, double r, double spd){
    x = x/determinebiggest(fabs(x),fabs(y)); y=y/determinebiggest(fabs(x),fabs(y));
    double rotationalratio = fabs(r)/(determinebiggest(fabs(x),fabs(y))+fabs(r));
    //above: very sketchy power distrubtion formula between rotation and translation
    for (int i = 0; i < sizeof(MAP); i++){
      //calculation for the power of each motor, see discord #design-ideas for formula
      MAP[i].mot = spd*127*((-x*MAP[i].sinV + y*MAP[i].cosV)*(1-rotationalratio) + rotationalratio*r);
      //above: rotationalratio code made even more sketchier, it doesnt even scale correctly I think
    };
  }
};

/*odometrycontroller: interface for ADI_Encoder to determine the position of the bot:
  - Rotational and translational data updating
*/
struct odometrycontroller{
  ADIEncoder* left;
  ADIEncoder* right;
  ADIEncoder* back;
  double ds, db; //offsets from center of rotation, not diameter
  odometrycontroller(ADIEncoder en[],double s, double b):ds(s),db(b){left = &en[0]; right = &en[1]; back = &en[2];}
  //updates position, should be triggered once every 10ms or so
  void updateposition(){
    double LD = rottodist(left->get_value(),STD_TWHEEL_RADIUS);
    double RD = rottodist(right->get_value(),STD_TWHEEL_RADIUS);
    double HD = rottodist(back->get_value(),STD_TWHEEL_RADIUS);//for calculating lateral shift we make a perpendicular line on our arc
    double relangle = (RD-LD)/(2*ds); //100% this part works
    double chordlength = 2*(RD/relangle)*sin(relangle/2); //85% this part works
    xG+=chordlength*cos(angleG)+HD*sin(angleG+(relangle/2)); //15% this and below work
    yG+=chordlength*sin(angleG)+HD*cos(angleG+(relangle/2));
    angleG = fmod((angleG+relangle),(2*M_PI)); //technically sketchy but not really still pls test
    left->reset(); //these resets seem to be not very reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
  }
  //keys position to a hardcoded target, use for wall allignments
  void key_position(double x, double y, double r){
    left->reset(); //these resets seem to be not very reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
    xG = x;
    yG = y;
    angleG = r;
  }
};
