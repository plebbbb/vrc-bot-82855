#include "global.hpp"
// This cpp is for the declration of extern objects from global.hpp

/*a tip making motorw arrays:
  it is suggested to not inverse motor direction, and instead figure out the
  correct vector direction for positive movement. This prevents potential issues
  with rotation.
*/


//********************************************************************************//
//Variables:
double speedmultiplier = 100; //IN PERCENT, 100 being 100%
double angleG = M_PI/2;
double xG = 0;
double yG = 0;
double xR = 0; //appears unused
double yR = 0; //appears unused
double estspd = 0;
double heading = angleG;
double xyaT[3] = {0,0,angleG};
double tgtangent = angleG;
bool anglemode;
bool GVT;
const int AXIS_COUNT = 0;

//********************************************************************************//
//Pascal's Triangle for Bezier calculations so we can skip manual computation
//this is the binomial factor thing
//I dont think we would do more than deg 10 anyways so its only up to 10 for now
const std::vector<short> Ptriangle[] = { //short cuz we never go past even 200
  {1},
  {1,1},
  {1,2,1},
  {1,3,3,1},
  {1,4,6,4,1},
  {1,5,10,10,5,1},
  {1,6,15,20,15,6,1},
  {1,7,21,35,35,21,7,1},
  {1,8,28,56,70,56,28,8,1},
  {1,9,36,84,126,126,84,36,9,1}
};

//********************************************************************************//
//ADIEncoder arrays:
//ADIEncoder format: pin 1, pin2, inversed or not
//Array format: Left, Right, Back
ADIEncoder odencoders[3] = {
  ADIEncoder({1,'C','D'} ,true),
  ADIEncoder({1,'B','A'},false),
  ADIEncoder({1,'F','E'},true)
};


//********************************************************************************//
//motorw arrays:
//motorw format: pin, inv dir, orientation, wheel size
motorw kiwimotors[] = {
  motorw(1,true,(M_PI*2)/3), //right motor
  motorw(2,true,(M_PI*4)/3), //left motor
  motorw(3,true,0) //Pure translational Motor
};

motorw xdrivemotors[] = {
  motorw(10,true,(3*M_PI)/4), //top right corner
  motorw(20,true,(M_PI)/4), //bottom right corner
  motorw(17,true,(7*M_PI)/4), //bottom left corner
  motorw(19,true,(5*M_PI)/4), //top left corner
};

//********************************************************************************//
//PID Variables

//********************************************************************************//
//PIDKvals format: Pk, Ik, Dk
double PIDKvals[][3] = {
  {1,0,0.1},     //direct distance PID
  {5,1.5,3},        //direct rotation PID for driver mode
  {4,0,0.25},       //direct rotation PID
  {2,0.05,1},       //heading offset PID
  {3,0,1.5},          //direct X/Y axis PID
};

//FOR REFRENCE: below is V1.215, where we had decent direct PID performance in line operation mode
/*
double PIDKvals[][3] = {
  {7.5,0.0005,2},     //direct distance PID
  {5,1.5,3},        //direct rotation PID for driver mode
  {3,0,2},       //direct rotation PID
  {2,0.05,1},       //heading offset PID
  {3,0,1},          //direct X/Y axis PID
}*/
//********************************************************************************//
/*PIDKvals format:
S-curve enable/disable, - turns the P output into an S curve. it's quite rough but does give us a smoother motion. works only for percentage based PID loops
fractional I enable/disable, - converts I into an asymptope equation where the I value approaches the I cap value
I hardstop at target enable/disable - sets i to 0 when target reacher to prevent the I value from overshooting*/
bool PIDSvals[][3] = {
  {false,false,true}, //standard
  {true,false,true}   //should be for motorFs and any speed PIDs
};

//********************************************************************************//
//PIDL values: PID hard limit, I value hard limit
double PIDLvals[][2] = {
  {100,50},
  {50,50}
};

//********************************************************************************//
//S curve VARIABLES
//Scurve possible Y range: 0-100, X range: 0-50
//Get your coefficients from desmos! https://www.desmos.com/calculator/aydhkipdkz
//Index 0: max spd, Index 1: slope, Index 2: horizontal offset, Index 3: vertical offset
//max spd should be always positive!
double Scurvevals[][4] = {
  {127,0.2,24.2,0},    //default settings from the desmos link
  {100,0.335,13,3},  //settings for the direction distance PID's stopping half
  {100,0.32,7,3}  //settings for the direction distance PID's starting half
};

//dualScurve wrapper, use Scurvevals to dictate curve types
dualScurve* curvesets[] = {
  new dualScurve(Scurvevals[0],Scurvevals[1]) //default desmos link, flipped past 50%
};


//********************************************************************************//
//Auton control stuff

//dataset for motion parameters
/*

*/
//data input scheme: array of position params, array of orientation params
//yes, this is a sketchy way of data input, and yes we are doing this because we dont have to make anything fancier
/*formatting scheme:
     the first index holds entire auton sequences, as in skills auton or alliance side auton
     the second index holds complete sets of parameters for motion instances, being a composize bezier curve with rotation and movement commands
     the third index [0] is the set of bezier transformations for each motion's composite bezier
         The first index of this transformation should be the current position, in the new desired direction, not the one from the last movement
         Composite bezier indexes: {X, Y, Angle at (x,y), angle transformation factor}
     the third index [1] is the set of rotation commands in each movement
        These are ordered from earliest to latest ranges
        In non-specified ranges, automatic angle optimization is enabled. It is recommended to enable this for the early part of each motion to prevent hitting things from large spins
     the third index [2] is to be the set of subsystem commands once those get implemented*/

std::vector<std::vector<std::vector<std::vector<double>>>> motionparams[] = {
  { //Test sequence
    { //Motion Set 1
      { //Composite bezier set 1
        {0,0,M_PI/2,3},
        {5,10,M_PI/2,5},
        {15,10,M_PI,2},
      },
      { //Rotational command array 1
        {M_PI/2+1,15,30},
        {M_PI,50,90},
        {2*M_PI,95,100}
      }
    },
    { //Motion Set 2
      { //Composite bezier set 2
        {15,10,3*M_PI/2,3},
        {5,10,3*M_PI/2,5},
        {0,0,3*M_PI/2,2},
      },
      { //Rotational command array 2
        {M_PI/2+1,15,30},
        {M_PI,50,90},
        {2*M_PI,95,100}
      }
    }
  }
};


//set of motion instances for actual auton processing
//input scheme: dualScurve*, compositebezier*/new compositebezier, orientationscheme*/new orientaitionscheme, iteration percentage factor
std::vector<motion> motionpaths = {
  motion( //Motion set 1
    curvesets[0],
    new compositebezier(motionparams[0][0][0]),
    new orientationscheme(motionparams[0][0][1]),
    5
  )
};

//********************************************************************************//
//PID controllers
//constructor scheme: PIDKvals, PIDSvals, PIDLvals, (OPTIONAL)dualScurve

//PID for base navigation
PID bPID[] = {
  PID(PIDKvals[0],PIDSvals[1],PIDLvals[1]), //direct distance PID
  PID(PIDKvals[2],PIDSvals[0],PIDLvals[1]), //direct rotation PID
  *new PID(PIDKvals[1],PIDSvals[0],PIDLvals[0]), //direct rotation PID for driver mode
  PID(PIDKvals[3],PIDSvals[0],PIDLvals[0]), //heading offset PID
  PID(PIDKvals[4],PIDSvals[0],PIDLvals[0]), //direct X axis PID
  PID(PIDKvals[4],PIDSvals[0],PIDLvals[0]), //direct Y axis PID
};

//PID e = PID(PIDKvals[0],PIDSvals[0],PIDKvals[0],curvesets[0]); //example setup for a motorF

//********************************************************************************//

//Control scheme configuration
//array format: left-right, forwards-back, clockwise-counterclockwise
controller_analog_e_t controlscheme[]{
  ANALOG_LEFT_X,
  ANALOG_LEFT_Y,
  ANALOG_RIGHT_X
};

//Control scheme featureset
//array format: enable absolute mode, enable angle hold
bool configoptions[]{
  false,
  true,
  false
};

//********************************************************************************//
//motorf array
motorf NBmotors[] = {
  motorf(PIDKvals[0],PIDSvals[0],PIDLvals[0],9,DIGITAL_UP)
};





//********************************************************************************//
//actual controllers
Controller ctrl = E_CONTROLLER_MASTER;
odometrycontroller odo = *new odometrycontroller(odencoders,Y_AXIS_TWHEEL_OFFSET,X_AXIS_TWHEEL_OFFSET);
basecontroller base = *new basecontroller(xdrivemotors);
coordcontroller mover = *new coordcontroller(base,bPID);
intakecontroller intakes{Motor(6),Motor(7,true),Motor(8),Motor(3,true),DIGITAL_L1, DIGITAL_L2}; //epic cheese momento. 7 is right intake
opcontrolcontroller useonlyinopcontrol = *new opcontrolcontroller(&base, controlscheme,&bPID[2],configoptions);
//********************************************************************************//
//functions:
double determinebiggest(double a, double b){
  double TA = a;
  double TB = b;
  if (isnanf(TA)) TA = 0;
  if (isnanf(TB)) TB = 0;
  if (a>b) return a;
  return b;
}

double isposorneg(double input){
  //there is 100% a better solution but this works so imma keep it for now
  //if someone figures it out pls change it
  return input/fabs(input); //its supposted to return 1 or -1, fyi
}

//WARNING, CRAD AND TRAD MEAN NOTHING. TEST EACH IMPLEMENTATION
double getrelrad(double crad, double trad){
  //note: untested but probably correct. Pls test
/*  if (fabs(crad-trad) > M_PI) return -(2*M_PI-fabs(crad-trad))*isposorneg(crad-trad);
  return (crad-trad);*/
  //double scrad = fmod(crad,M_PI*2);
//  double strad = fmod(trad,M_PI*2);
  double rdval = (trad-crad);
  if (rdval < -M_PI) return rdval+(M_PI*2);
  if (rdval > M_PI) return -((M_PI*2)-rdval);
  return rdval;
}

//this is pretty stupid
double rottodist(double rad, double radius){
  return rad*radius;
}

//this is solely for convience -
//NGL tho for everyone who already has to use radians, just get used to them
double degtorad(double deg){
  return fmod(deg*(M_PI/180),2*M_PI);
};

//recursive approach to computing factorials
double factorial(double n){
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

//determine smallest functions
double determinesmallest(double a, double b){
  double TA = a;
  double TB = b;
  if (isnanf(TA)) TA = 0;
  if (isnanf(TB)) TB = 0;
  return (TA > TB) ? TB : TA;
}

//determine smallest functions, absolute edition
double determinesmallestA(double a, double b){
  return (fabs(a) > fabs(b)) ? b : a;
}

//jank copy array
 void arraycopy(double tgt[], double ref[], int size){
  for (int i = 0; i < size; i++){
    tgt[i] = ref[i];
  }
}

//checks array equality
 bool isarrsame(double a[], double b[], int size){
   for (int i = 0; i < size; i++){
     if (a[i]!=b[i]) return false; //peak jank, if their difference isnt 0 they arent the same
   }
   return true;
 }

//********************************************************************************//
//Below: doesnt work b/c header issues
/*void basecontrollerdebug(basecontroller a){
  lcd::print(0, "TR motor: %f",a.vals[0]);
  lcd::print(1, "BR motor: %f",a.vals[1]);
  lcd::print(2, "BL motor: %f",a.vals[2]);
  lcd::print(3, "TL motor: %f",a.vals[3]);
  lcd::print(5, "Rot Ratio: %f",a.rotationalratio);
}
void trackingwheeldebug(odometrycontroller a) {
lcd::print(0,"Left Encoder: %d", (a.left->get_value()));
lcd::print(1,"Right Encoder: %d", (a.right->get_value()));
lcd::print(2,"Back Encoder: %d", (a.back->get_value()));
}*/
void odometrycontrollerdebug(){
lcd::print(0,"X: %f",xG);
lcd::print(1,"Y: %f", yG);
lcd::print(2,"Angle: %f", angleG);
//lcd::print(3,"Est Spd: %d in/s", (int)estspd);
//lcd::print(4, "Est Heading: %f", heading);
}
