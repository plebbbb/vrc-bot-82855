#include "main.h"
#include "utilityfunctions.hpp"
#include "globalvar.hpp"
using namespace pros;
#pragma once

  //curveS: a single S curve
  //Constraints(suggested): range: 0 to 50, upwards.  max height 100;
  //TBD, maybe actually calculate proper motion curves later and emulate them with curveS
struct curveS{
  double* vars;
public:
  curveS(double arr[]){vars = arr;}
  double getval(double pos){
    return (vars[0]-vars[3])/(1+pow(M_E,-vars[1]*(pos-vars[2]))) + vars[3];
  }
};

  //dualScurve: a set of 2 S curves made to aproximate what motion profiling might look like
  /*dualScurve should extend curveS, also should be an interface ngl so we can use both one or two
  the current pointer access method is ok for these b/c they dont store anything motor-specific,
  its literally just a formula*/
  //TBD: scale dualScurve to actual time values. an S curve is supposed to be relative to time, not displacement like we use it
  //Even right now it still outperforms raw PIDs, but we will get better performance if we adjust it
  //To do that, we have to unconvert the area(the displacement) of the curve into its corrospondent speed in the S curve
  //this requires some calc stuff so it's gonna take a while
struct dualScurve{
  curveS* a;
  curveS* b;
  dualScurve(curveS c, curveS d){a=&c; b=&d;}
  dualScurve(curveS c){a=&c;}
  double getval(double ps){
    double pos = fabs(ps);
  //  lcd::print(1,"given value: %f", pos);
    if (pos < 50) {
      //lcd::print(2,"return value: %f", a->getval(pos));
      return a->getval(pos);
    }
    else if (!b) {
    //___int_least64_t_defined  lcd::print(2,"return value: %f", a->getval(100-pos));
      return a->getval(100-pos);
    }
  //  lcd::print(2,"return value: %f", b->getval(100-pos));
    return b->getval(100-pos);
    //for the part past 50%, we flip the curveS, so that we only need one curve from 0-50 for each different profile
  };
};

  //beziernp: a raw N-point bezier curve
  /*This is a full on proper n-point bezier curve, where we can add
  as many transformations as we want but only 2 garanteed target locations
  */

struct beziernp{
    double (*coords)[2];
    int size; //coordinate size
    //params: Coordinates of NP bezier, amount of coordinates(not an index)
    //and yes, I actually need the size thing cuz pointers dont pass along the actual size
  public:
    beziernp(double points[][2], int sie):size(sie){
  	  coords = points; //fancy speed and heading configurations precomputed in compositebezier to reduce load
      for(int i = 0; i <size; i++){
        for (int o = 0; o < 2; o++){
          printf("\nCoords (%d,%d) : %f",i,o,coords[i][o]);
        }
      }
    }
    //variation two, probably higher chance of working than the other option, but probably
    //a lot more taxing, we may actually hit performance issues from this once
    //https://www.desmos.com/calculator/xlpbe9bgll
    void getvalF(double t){
      double x = 0; double y = 0;
      for (int i = 0; i < size; i++){
        double ccfactor = getCCF(t,i);
        x+=ccfactor*coords[i][0];
        y+=ccfactor*coords[i][1];
      }
      xyaT[0] = x;
      xyaT[1] = y;
    }
    //formatting: t is iteration position, k is index, v is coordinate
    //TBD: precompute these as well, we just gotta set a fixed point resolution limit then throw this in eclipse or something,
    //and have it print out a bunch of datasets which we paste in. We probably have enough memory
  //private:
    double getCCF(double t, double k){
      return (Ptriangle[size-1][k])*pow((1-t),size-1-k)*pow(t,k); //to save performance we calc this once per coord
    }
};

//composite N-point bezier curve, currently set up to only go to degree 4. Custom points dataset to be implemented
//for the moment, active speed and trajectory controls are going to be possible by code design, but we will use precalculated values for now until odometrycontroller's estimations are fixed
//to implement that, each bezier must be generated at runtime upon completion of the last bezier, using the
//velocity and heading values as subsitutes for second point of the bezier.
class compositebezier{
  std::vector<beziernp> genarr; //WARNING: appending to this thing can kill your pointers if it has to resize itself to fit in the appended element
public:
  int size; //could be a pointer at the moment I think this is just wasting memory but at the same time a pointer might be bigger than an int
  //This constructor is for hard set degree 4 composites
  //data params: x, y, angle
  //use the length value(raw size() output), not max index for size
  compositebezier(double** data, int len):size(len){
    for(int i = 0; i < size-1; i++){ //tbd change to an std::vector.insert if possible
      double (*params)[2] = new double[4][2];
      double pr[][2] = { //TBD throw this directly into the beziernp instead of creating it externally
        {data[i][0],data[i][1]},
        {cos(data[i][2]),sin(data[i][2])}, //Speed configuration will be dealt with externally in the speed controller
        {data[i+1][0]-cos(data[i+1][2]),data[i+1][1]-sin(data[i+1][2])},  //so basically we will access params and multiply these values to do the speed transformation
        {data[i+1][0],data[i+1][1]}
      };
      for (int x = 0; x < 4; x++){
        for (int y = 0; y < 2; y++){
          params[x][y] = pr[x][y];
          //printf("PARAMS: %d, %d : %f",x,y,params[x][y]);
        }
      }
      genarr.push_back(*(new beziernp(params,4)));
    }
  }

  //if intaking a std::vector
  compositebezier(std::vector<double> data[], int len):size(len){
    for(int i = 0; i < size-1; i++){ //tbd change to an std::vector.insert if possible
      double (*params)[2] = new double[4][2];
      double pr[][2] = { //TBD throw this directly into the beziernp instead of creating it externally
        {data[i][0],data[i][1]},
        {cos(data[i][2]),sin(data[i][2])}, //Speed configuration will be dealt with externally in the speed controller
        {data[i+1][0]-cos(data[i+1][2]),data[i+1][1]-sin(data[i+1][2])},  //so basically we will access params and multiply these values to do the speed transformation
        {data[i+1][0],data[i+1][1]}
      };
      for (int x = 0; x < 4; x++){
        for (int y = 0; y < 2; y++){
          params[x][y] = pr[x][y];
          //printf("PARAMS: %d, %d : %f",x,y,params[x][y]);
        }
      }
      genarr.push_back(*(new beziernp(params,4)));
    }
  }
  //safe, no deletion mode. In retrospect this is probably sufficient. Each beziernp is like 8 bytes so we shouldn't see any issues given that we have like 40MB of usable memory
  void updvalnd(double pos){
    short ind = floor(pos); //tbd delete for memory?
    genarr[ind].getvalF(pos);
  };
};


struct motion{
  compositebezier a;
  std::vector<double>* val;
  //BELOW: SHOULD BE WHAT AXIS_COUNT IS
  double updvals[0]; //this entire passing through of the motorF values is kinda stupid, also this should be whatever AXIS_COUNT is
  double Enablethreshold;
  double Disablethreshold;
  dualScurve* DSC;
  double length; //this is raw length of all params, not max index
  motion(std::vector<double> e[], int lth, double SV, double EV, dualScurve sc):a(e,lth){ //SV/EV: index at which to enable/disable angle optimization
    val = e;
    DSC = &sc;
    //a = new compositebezier(val,lth);
    length = lth;
    for(int i = 0; i < lth; i++){
      printf("\nVal: %f",e[0][i]);
    }
    Enablethreshold=(SV/lth)*100;
    Disablethreshold=(EV/lth)*100;
  }
  void update(double perc){
    double va = (determinesmallest(perc/100,1))*length;
    int vr = ceil(va)-1; //math.ceil(1.00) = 1
    a.updvalnd(va);
    xyaT[2] = val[vr][3]; //target angle confirmation
    for(int i = 0; i < AXIS_COUNT; i++){
      updvals[i] = val[vr][i+4]; //first 3 indexes are x, y, heading angle, real angle and so are ignored
    }
  }
};
/*PID: generic PID system*/
//NOTE: DEFAULT TGT = 0
struct PID{
//private:
  /*Integral mode configurations:
  false: Direct I scaling - 100% of I is added each cycle
  true: Asymptope I - I approaches the max*/
  bool Imode = 0;

  /*Proportional mode configurations
  false: Raw input
  true: External S curve, percent to target based
  */
  bool Pmode = 0;

  //Izerocutoff: turns I to 0 when target reached
  bool Izerocutoff = true;

  //Pk, Ik, Dk, and I fraction array
  double* ratios;

  //selected dualScurve for Pmode = 1
  dualScurve* Scurve;

  //the max limits for the loop
  double maxIlimit;
  double maxlimit;

  //P, I, D, position, and target array
  double PIDa[4] = {0};

  double lasterror = 0;
//public:
  PID(double scalers[], bool ms[], double limits[]){
    ratios = scalers; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = limits[0]; maxIlimit = limits[1];
  }
  //note: if dualScurve is to be used, input percentage to target values
  PID(double scalers[], bool ms[], double limits[], dualScurve curve){
    ratios = scalers; Scurve = &curve; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = limits[0]; maxIlimit = limits[1];
  }
  //sets a new target for the loop w/o resetting PIDa
  void set_tgt_soft(double tgt){
    PIDa[3] = tgt;
  }
  //sets a new target for the loop and resets PIDa
  void set_tgt_clean(double tgt){
    PIDa[0]=0;PIDa[1]=0;PIDa[2]=0; //this is stupid
    PIDa[3] = tgt;
  }
  double update(double in){
    double err = (PIDa[3]-in);
    if (Pmode) PIDa[0] = isposorneg(err)*Scurve->getval(err);
    else PIDa[0] = err;
    if (Imode) PIDa[1] += (err-PIDa[1])/ratios[3];
    else PIDa[1] += err;
    if (Izerocutoff == true && round(in*10) == 0) PIDa[1] = 0;
    if (fabs(PIDa[1]) > maxIlimit) PIDa[1] = isposorneg(PIDa[1])*maxIlimit;
    PIDa[2] = err-lasterror;
    lasterror = err;
    if (isnanf(PIDa[0])) PIDa[0] = 0;
    if (isnanf(PIDa[1])) PIDa[1] = 0;
    if (isnanf(PIDa[2])) PIDa[2] = 0;
  //  lcd::print(3,"P: %f", PIDa[0]);
    double final = PIDa[0]*ratios[0] + PIDa[1]*ratios[1] + PIDa[2]*ratios[2];
    //lcd::print(4,"I: %f", PIDa[1]);
    //lcd::print(5,"D: %f", PIDa[2]);
    return isposorneg(final)*determinesmallest(fabs(final),maxlimit);
  };
};
