#include "main.h"
#include "utilityfunctions.hpp"
using namespace pros;
#pragma once

//curveS: a single S curve
//Constraints(suggested): range: 0 to 50, upwards.  max height 127;
//TBD, maybe actually calculate proper motion curves later and emulate them with curveS
class curveS{
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
struct dualScurve{
  curveS* a = NULL;
  curveS* b = NULL;
  dualScurve(curveS c, curveS d){a=&c; b=&d;}
  dualScurve(curveS c){a=&c;}
  double getval(double pos){
    if (pos < 50) return a->getval(pos);
    else if (b == NULL) return a->getval(50-pos);
    return b->getval(50-pos);
    //for the part past 50%, we flip the curveS, so that we only need one curve from 0-50 for each different profile
  };
};

/*PID: generic PID system*/
//NOTE: HAS NOT BEEN TESTED PLS TEST
//ANOTHER NOTE: DEFAULT TGT = 0
class PID{
private:
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
public:
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
    if (Pmode) PIDa[0] = isposorneg(err)*Scurve->getval(fabs(err));
    else PIDa[0] = err;
    if (Imode) PIDa[1] += (err-PIDa[1])/ratios[3];
    else PIDa[1] += err;
    if (Izerocutoff && in == 0) PIDa[1] = 0;
    if (fabs(PIDa[1]) > maxIlimit) PIDa[1] = isposorneg(PIDa[1])*maxIlimit;
    PIDa[2] = err-lasterror;
    lasterror = err;
    double final = PIDa[0]*ratios[0] + PIDa[1]*ratios[1] + PIDa[2]*ratios[2];
    return isposorneg(final)*determinebiggest(fabs(final),maxlimit);
  };
};
