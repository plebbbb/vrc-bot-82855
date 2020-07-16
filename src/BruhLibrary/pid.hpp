#include "main.h"
#include "utilityfunctions.hpp"
#include "globalvar.hpp"
using namespace pros;
#pragma once

  //curveS: a single S curve
  //Constraints(suggested): range: 0 to 50, upwards.  max height 127;
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
struct dualScurve{
  curveS* a;
  curveS* b;
  dualScurve(curveS c, curveS d){a=&c; b=&d;}
  dualScurve(curveS c){a=&c;}
  double getval(double ps){
    double pos = fabs(ps);
  //  lcd::print(1,"given value: %f", pos);
    if (pos < 50) {
      lcd::print(2,"return value: %f", a->getval(pos));
      return a->getval(pos);
    }
    else if (!b) {
      lcd::print(2,"return value: %f", a->getval(100-pos));
      return a->getval(100-pos);
    }
    lcd::print(2,"return value: %f", b->getval(100-pos));
    return b->getval(100-pos);
    //for the part past 50%, we flip the curveS, so that we only need one curve from 0-50 for each different profile
  };
};

  //beziernp: a candidate approach for our path finding solution
  /*This is a full on proper n-point bezier curve, where we can add
  as many transformations as we want but only 2 garanteed target locations
  */
class beziernp{
    double (*coords)[2];
    int size; //coordinate size
    double CCM;
    //params: Coordinates of NP bezier, amount of coordinates(not an index)
    //and yes, I actually need the size thing cuz pointers dont pass along the actual size
    beziernp(double points[][2], int sie):size(sie){
  	  coords = points;
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
    //TBD: precompute these as well, we just gotta set a fixed point resolution limit
    double getCCF(double t, double k){
      return (Ptriangle[size-1][k])*pow((1-t),size-1-k)*pow(t,k); //to save performance we calc this once per coord
    }
};
//compositebezier: another candidate approach for path finding
/*This is a piecewise bezier curve approach using beziernp instances, of which
will have 4 default offset points each, two for end points, two for heading targets*//*
struct compositebezier{
  std::vector<beziernp> beziers; //ngl shoulda started running vectors a lot sooner
  int size;
  //arr config: include the bezier from your current positon too
  compositebezier(beziernp arr[]){
    size = sizeof(arr)/sizeof(beziernp);
    for (int i = 0; i < size; i++) beziers.push_back(arr[i]);
  }
  //mvcoords config - do not include the current position, only future positions
  //new bezier curve is generated using current point
  compositebezier(double mvcoords[][3]){
    size = sizeof(mvcoords)/sizeof(double[3])+1;
    beziers.push_back(beziernp(mvcoords[0],mvcoords[0]));
    for (int i = 1; i < size; i++){
      beziers.push_back(beziernp(mvcoords[0],mvcoords[1]));
    }
  }
};
*/

/*PID: generic PID system*/
//NOTE: HAS NOT BEEN TESTED PLS TEST
//ANOTHER NOTE: DEFAULT TGT = 0
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
