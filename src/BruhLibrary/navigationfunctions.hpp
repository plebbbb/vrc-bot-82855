#pragma once
#include "global.hpp"


struct odometrycontroller{
  ADIEncoder* left;
  ADIEncoder* right;
  ADIEncoder* back;
  double ds, db; //offsets from center of rotation, not diameter
  odometrycontroller(ADIEncoder en[],double s, double b):ds(s),db(b){left = &en[0]; right = &en[1]; back = &en[2];}
  //below: lazier, revised edition. Seperates into local and global coordinate conversions instead of all in one
  void posupdv2(){
    double xLN, yLN;
    double LD = rottodist(degtorad(left->get_value()),STD_TWHEEL_RADIUS);
    double RD = rottodist(degtorad(right->get_value()),STD_TWHEEL_RADIUS);
    double HD = rottodist(degtorad(back->get_value()),STD_BTWHEEL_RADIUS);
    double rang = ((RD-LD)/(ds*2));
    if (rang == 0){
      xLN = HD;
      yLN = RD;
    }
    else {
      yLN = 2*sin(rang/2)*(LD/rang + ds);
      xLN = 2*sin(rang/2)*(HD/rang + db);
    }
    double avang = angleG+(rang/2);
    double xC = yLN*cos(avang)+xLN*cos(avang-(M_PI/2)); //conversion to global coords
    double yC = yLN*sin(avang)+xLN*sin(avang-(M_PI/2));
    xG+=xC;
    yG+=yC;
    angleG+=rang;
    if (angleG > (M_PI*2)) angleG = angleG - (M_PI*2);
    if (angleG < 0) angleG = angleG + (M_PI*2);
    estspd = sqrt(xLN*xLN + yLN*yLN)*100; //x100 to convert to in/s from in/10ms
    if (xC != 0) heading = fmod(atan2(yC,xC),(2*M_PI));
    else if (yC > 0) heading = M_PI/2; //if moving directly up
    else if (yC < 0) heading = (3*M_PI)/2; //if moving directly down
    else heading = 0; //if not moving
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
  }
  //keys position to a hardcoded target, use for wall allignments
  void key_position(double x, double y, double r){
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
    xG = x;
    yG = y;
    angleG = r;
  }
};

/*coordcontroller: a wrapper for basecontroller to intepret coordinate grid inputs
    While it 100% is kinda stupid to hsave this many layers, this is done to allow
    a bit of distinction between each layer of sortware interaction. This way,
    troubleshooting, as well as understanding the code can be a bit easier.
*/
//TBD: refactor coordcontroller into multiple processing functions per update function. It is bloaty, at least the independent line update function.
struct coordcontroller{
  basecontroller* mBase;
  double oldxyat[3] = {0,0,0};
  double distance;
  PID* axiscontrollers; //the initial plan called for 3 PID controllers to allow for smooth motion curves, but for now we have a direct line approach
  //double* xyaT; //we are gonna try a potentially stupid approach, where we dont call coordcontroller but instead change the tgt coords directly
  coordcontroller(basecontroller *a, PID b[]){mBase = a; axiscontrollers = b;}
  /*returns true when target is reached
    potential camera implementation: overload update with version that replaces r and perp with camera controls
    this overload would input the desired color profile that the camera is looking for.
    note that constructor must be updated for this*/
  //TBD: the current oldxyat system is overkill. Simplify AOM update checks into a saner configuration.
  bool update(){
    //double yO = 0;
    //note that it isnt really nescessary, but made to minimize the risk of swaying in circles, it itself is disabled
    //past a certain point for safety's sake, although it is likely isn't gonna do anything weird when we get close to the target
    double xGD = (xyaT[0]-xG); //global x distance
    double yGD = (xyaT[1]-yG); //global y distance
    double dist = sqrt(xGD*xGD+yGD*yGD);
    //the whole idea of this is to figure out what direction we moved in so we can auto orient ourselves in the most efficient direction.
    //the current implementation uses instanious rates of change to do this, meaning that if we dont move things get messed up by divison by zero.
    if (!isarrsame(xyaT, oldxyat, 3)) {
      distance = dist;
      double sl = determinesmallest(100, 0.75*distance+20); //linear formula for s curve speed limit, def not realistic but its close enough
      axiscontrollers[0].Scurve->a.vars[0] = sl;
      axiscontrollers[0].Scurve->b.vars[0] = sl; //could be jank if there isnt a different downward S curve
      //shouldnt be a problem tho cuz I dont think we ever deal with that case. this is here if u get memory errors
      //for(int i = 0; i < 3; i++){oldxyat[i] = xyaT[i];}
      arraycopy(oldxyat, xyaT, 3); //TEST TO SEE IF THIS ACTUALLY WORKS NOW
    }
    double xD = 0;
    double yD = 0;
    double xCC = 0; //independent X axis PID
    double yCC = 0; //independent Y axis PID
    double rD = 0; //VERY janky figure out better solution than a hard multiplier
    //we switch modes into an axis specific PID mode once we get close to prevent circular movement
    //this if statement can be optimized to just overwrite the GD variables instead of making the updvals, but this is more readable
    xCC = axiscontrollers[4].update(-xGD); //neg b/c PID responds to offset to target, not other way around
    yCC = axiscontrollers[5].update(-yGD);
    if (isnanf(xCC)) xCC = 0;
    if (isnanf(yCC)) yCC = 0;
    if (dist < 2.5){ //trigger x-y specific PID on activation
      xD = xGD*cos(getrelrad(angleG-M_PI/2,0))+yGD*cos(getrelrad(angleG,M_PI)); //relative distances to target
      yD = yGD*sin(getrelrad(angleG,M_PI))+xGD*sin(getrelrad(angleG-M_PI/2,0)); //relative distances to target
      rD = axiscontrollers[1].update(-7.5*(getrelrad(angleG,xyaT[2])));
    }else{
      xD = xCC*cos(getrelrad(angleG-M_PI/2,0))+yCC*cos(getrelrad(angleG,M_PI));
      yD = yCC*sin(getrelrad(angleG,M_PI))+xCC*sin(getrelrad(angleG-M_PI/2,0));
      rD = axiscontrollers[1].update(-20*(getrelrad(angleG,xyaT[2])));
    }

    //if ((sqrt(pow(xD,2)+pow(yD,2))) > 10) yO = axiscontrollers[3].update(getrelrad(heading, atan2(xG-xyaT[0],yG-xyaT[1])));
    //PID offset system if the motors aren't 100% correct orientation wise. May cause potential spinning issues near target
    //Below: Sketchy, and most likely redundent math to account for yO in the local coordinate system
    //xD+=yO*sin(atan2(xD,yD));
    //yD+=yO*cos(atan2(xD,yD));
    if(isnanf(rD)) rD = 0;
    double LPID = fabs(axiscontrollers[0].update(fabs(100*(dist/distance)))); //axiscontrollers is now on a percent basis
    if(isnanf(LPID)) LPID = 0; //this shouldnt have to exist, it only means that the PID is borked somewhere
    //double LPID = fabs(axiscontrollers[0].update(dist));
    //double LPID = fabs(xCC) + fabs(yCC);
    double RPID = determinesmallest(fabs(rD),25);
    double speed = determinesmallest(70, LPID+RPID);
/*    lcd::print(1,"S curve cap:%f",axiscontrollers[0].Scurve->b->vars[0]);
    lcd::print(3,"Speed: %f",speed);
    lcd::print(4,"percent tgt: %f", 100*(dist/distance));
    lcd::print(5,"linear PID: %f", LPID);
    lcd::print(6,"rotational PID: %f", RPID);*/
    mBase->vectormove(xD,yD,rD,speed);
        //less than 2 inch distance, and less than 2% angle offset to commit to next stage
    if (round(dist/4 + fabs(rD/(M_PI*2))*25) == 0) return true;
    else return false;
  }

  //this variation is for usage with motionpaths, where axiscontrollers merely maintains the speed target given by TSP
  //automatic angle optimization doesn't self check if it's divisible here, so it's dependent on AOM being disabled at the very end of each movement
  bool updateMP(){
      double xGD = (xyaT[0]-xG); //global x distance
      double yGD = (xyaT[1]-yG); //global y distance
      double dist = sqrt(xGD*xGD+yGD*yGD);
      double xD = 0;
      double yD = 0;
      double xCC = 0; //independent X axis PID
      double yCC = 0; //independent Y axis PID
      double rD = 0; //VERY janky figure out better solution than a hard multiplier and also stop using the same variable for both difference and output power
      //we switch modes into a direct axis specific PID mode once we get close to prevent circular movement
      //this if statement can be optimized to just overwrite the GD variables instead of making the updvals, but this is more readable
      xCC = axiscontrollers[4].update(-xGD); //neg b/c PID responds to offset to target, not other way around
      yCC = axiscontrollers[5].update(-yGD);
      //angle optimization to ensure we are always having our wheels face 45 degrees during the middle of movement
      if (anglemode) rD = AOM_P_VAL*determinesmallestA(
          determinesmallestA(getrelrad(angleG,tgtangent),getrelrad(angleG,tgtangent+M_PI/2)),
          determinesmallestA(getrelrad(angleG,tgtangent+M_PI),getrelrad(angleG,tgtangent+(3*M_PI/2)))
      );
      else rD = getrelrad(angleG,xyaT[2]); //rotationmode is held in fixed segements of the movement path and released near the target to the real final angle
      if (isnanf(xCC)) xCC = 0; //honestly screw nan I would expect stuff to be so cheese that it defaults to a 0
      if (isnanf(yCC)) yCC = 0;
      if(isnanf(rD)) rD = 0;
      if (dist < 0.5){ //trigger x-y specific PID on activation
        xD = xGD*cos(getrelrad(angleG-M_PI/2,0))+yGD*cos(getrelrad(angleG,M_PI)); //relative distances to target
        yD = yGD*sin(getrelrad(angleG,M_PI))+xGD*sin(getrelrad(angleG-M_PI/2,0)); //relative distances to target
        rD = axiscontrollers[1].update(-7.5*(rD));
      }
      else{
        xD = xCC*cos(getrelrad(angleG-M_PI/2,0))+yCC*cos(getrelrad(angleG,M_PI));
        yD = yCC*sin(getrelrad(angleG,M_PI))+xCC*sin(getrelrad(angleG-M_PI/2,0));
        rD = axiscontrollers[1].update(-20*(rD));
      }
      //double angleoverride = GVT*(rD/(rD+xD+yD));
      //printf("xT: %f    yT: %f", xyaT[0],xyaT[1]);
      //printf("tangent: %f   AOM: %d", tgtangent, anglemode);
      //printf("xD: %f   yD: %f");
      mBase->vectormove(xD,yD,rD,GVT+10);
      //less than 2 inch distance to commit to next stage, angle only relevant if rotationmode disabled
      switch(anglemode){ //this is some janky ass logic but it should work
        case 0: //if (round(fabs(rD/(M_PI*2))*25) != 0) break; //if it is within angle tolerances pass through to case 1, only when we have a defined rotation target
        case 1: if (round(dist/4) == 0) return true; //case if angle optimization is enabled, checks if bot within position tolerances
      }
      return false;
    }
  };
