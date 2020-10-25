#include "main.h"
#pragma once
using namespace pros;
  /*it's throwing code errors but thats cuz we have a screwed up header structure where
  if we include this into the main stream with global.hpp, we would have a recursive loop
  of hpp files
  it works in the actual code b/c we include it after global.hpp in main.cpp those*/
  //if u wanna temparaly remove those errors so you can check for actual logic issues,
  //uncomment the line below:
#include "globalvar.hpp"
  //this is ripped straght from the old code. Ik it works but its still super sketchy
  //also we may want to work on the delays
void autonselection(){
  int msDelay = 50; // Delay in ms
  ctrl.clear();

  // Main Loop
  while(true){
    delay(msDelay);

    // When unconfirmed
    while(!confirmedauton){
      if(ctrl.get_digital_new_press(DIGITAL_LEFT)){
        selectedauton--;
        if(selectedauton < 0){
          selectedauton = 4;
        }
      }
      else if(ctrl.get_digital_new_press(DIGITAL_RIGHT)){
        selectedauton++;
        if(selectedauton > 4){
          selectedauton = 0;
        }
      }

      // A to confirm selection
      else if(ctrl.get_digital_new_press(DIGITAL_A)){
        confirmedauton = true;
      }
      /*if (selectedauton != lastselectedauton){
      delay(msDelay);
      switch(selectedauton){} ?? */
      delay(msDelay);
      //}
    }

    ctrl.clear();
    delay(msDelay);
    ctrl.set_text(0, 0, "AUTON SELECTED");
    delay(msDelay);
    ctrl.set_text(1, 0, "PRESS B TO confirm");
    delay(msDelay);
    ctrl.set_text(2, 0, "PRESS A TO CANCEL");

    bool bCheck = false; // for double break

    // When confirmed
    while(confirmedauton){

      // If auton select cancelled
      if(ctrl.get_digital_new_press(DIGITAL_A)){
        ctrl.clear();
        delay(msDelay);
        ctrl.set_text(0, 0, "SELECT AUTON");
        delay(msDelay);
        ctrl.set_text(2, 0, "PRESS A TO CONFIRM");
        delay(msDelay);
        confirmedauton = false;
      }

      // If confirmed
      else if (ctrl.get_digital_new_press(DIGITAL_B)){
        bCheck = true;
        break;
      }
    }
    if(bCheck){
      break;
    }
    delay(msDelay);
  }
  ctrl.clear();
  ctrl.set_text(0, 0, "PROGRAM START");
}
