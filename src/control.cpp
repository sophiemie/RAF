#include <Arduino.h>
#include "proj_types.h"
#include "robot.h"
#include "IRline.h"

extern IRLine_t IRLine;

void control(robot_t& robot)
{
    robot.tis = millis() - robot.tes;

    // Rules for the state evolution
     if(robot.state == 0 && robot.LastTouchSwitch && !robot.TouchSwitch) {
      robot.rel_s = 0;
      robot.setState(1);

    } else if (robot.state == 1 && robot.TouchSwitch) {
      robot.setState(2);

    } else if(robot.state == 2 && robot.tis > 600) { // 100
      robot.rel_s = 0;
      robot.setState(3);

    } else if(robot.state == 3 && robot.rel_s < -0.12) {
      robot.rel_theta = 0;
      robot.setState(4);

    //} else if(robot.state == 4 && robot.rel_theta > radians(90) && IRLine.total > 1500) {
   // } else if(robot.state == 4 && robot.rel_theta > radians(170)) { BEFORE
    } else if(robot.state == 4 && robot.rel_theta > radians(90)) {    
      robot.setState(5);
      IRLine.crosses = 0;

    } else if(robot.state == 5 && IRLine.crosses >= 5) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);

    } else if(robot.state == 6 && robot.rel_theta < radians(-45) && IRLine.total > 1500) {
      // else if(robot.state == 6 && robot.rel_theta < radians(-70) && IRLine.total > 1500) {
      robot.setState(7);

    } else if(robot.state == 7 && robot.tis > 1300) { // 2000
      robot.rel_s = 0; // we add that
      IRLine.crosses = 0;
      robot.setState(8);

    } else if(robot.state == 8 && robot.rel_s < -0.05) // && robot.tis > 2000) {
    {
      IRLine.crosses = 0;
      robot.rel_theta = 0;
      robot.setState(9);

    } else if(robot.state == 9 && robot.rel_theta < radians(-60)){ //&& IRLine.total > 1500) {
    //} else if(robot.state == 9 && robot.rel_theta < radians(-70) && IRLine.total > 1500) {    
      robot.setState(10);
      IRLine.crosses = 0;
    
    } else if(robot.state == 10 && IRLine.crosses >= 5) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(11);
    
    
    } else if(robot.state == 11 && IRLine.crosses >= 1) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(12);

    /*} else if (robot.state == 12  && robot.rel_theta > radians(30) && IRLine.total > 1500)
    {
      robot.rel_s = 0;
      robot.rel_theta = 0;      
      robot.setState(13); */

    //} else if(robot.state == 12 && robot.tis > 400 && robot.TouchSwitch) {/// from 1 and 2
    //} else if(robot.state == 12 && robot.tis > 7000 && robot.TouchSwitch) {///  Mit vorherigem TH // mit jetzigem 5000
    } else if(robot.state == 12 && robot.rel_theta > radians(60) ) { // 45
      robot.rel_theta = 0;
      robot.rel_s = 0;
      robot.setState(13);

     } else if(robot.state == 13 && robot.tis > 2000){  // 1000
      robot.rel_theta = 0;
      robot.rel_s = 0;
      robot.setState(14);

    } else if(robot.state == 14 && robot.TouchSwitch) {///  Mit vorherigem TH // mit jetzigem 5000
      robot.rel_theta = 0;
      robot.rel_s = 0;
      robot.setState(15);
      

    

    } else if(robot.state == 15 && robot.rel_s < -0.09)  // -0.04
      {
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(16);
      // just turning 90 degrees??
    //  } else if(robot.state == 14 && robot.rel_theta > radians(45) && IRLine.total > 1500) {  // Mit vorherigem Threshhold
   
    } else if(robot.state == 16 && robot.rel_theta > radians(30) && IRLine.total > 1500) {  

      IRLine.crosses = 0;
      robot.setState(17);   

    //////////////////////////////////////////

     } else if(robot.state == 17 && IRLine.crosses >= 1) {  // Vorher 2 // Rechts
      robot.rel_s = 0;
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(18);
      } else if(robot.state == 18 && IRLine.crosses >= 1) { // Links
      robot.rel_s = 0;
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(19);
      } else if(robot.state == 19 && IRLine.crosses >= 4) { ///// Sonst überspringt er 18??? vorher 2
      //robot.rel_s = 0;
      //robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(20);
      } else if(robot.state == 20 && IRLine.crosses >= 1) { // 2
      //robot.rel_s = 0;
      //robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(21);
 
      } else if(robot.state == 21 && IRLine.crosses >= 1) { // 2
      robot.rel_s = 0;
      //robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(22);
 
    

      } else if(robot.state == 22 && robot.rel_s < -0.09) {
      //robot.rel_s = 0;
      //robot.rel_theta = 0;
      IRLine.crosses = 0;
      //robot.setState(23);
 
     /* } else if(robot.state == 22 && IRLine.crosses >= 1) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(23); */

      } else if(robot.state == 23 && robot.tis > 9000) { // versuchen den schneller wegzukriegen, vorher 3000
        IRLine.crosses = 0;
        robot.rel_s = 0;
        robot.rel_theta = 0;
        robot.setState(24);


    } else if (robot.state == 202 && robot.tis > robot.T1) {
      robot.setState(200);
    }

    // Actions in each state
    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      robot.setRobotVW(0, 0);
    
    } else if (robot.state == 1) {  // Go: Get first box
      robot.solenoid_state = 0;
      robot.followLineLeft(IRLine, 0.15, -0.025); // 0.2, -0.05
      // if the request is older than 1000 ms repeat the request

    } else if (robot.state == 2) { // Turn Solenoid On and Get the Box
      robot.solenoid_state = 1;
      //if (robot.tis < 2000) 
      robot.followLineLeft(IRLine, 0.2, -0.05); // 0.1
    
    } else if (robot.state == 3) {  // Go back with the box
      robot.solenoid_state = 1;
      robot.setRobotVW(-0.1, 0);
      
    } else if (robot.state == 4) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      robot.setRobotVW(0, 2.5);
      
    } else if (robot.state == 5) {  // long travel to the box final destination
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.025);
      //robot.followLineRight(IRLine, 0.1, -0.05);
    
    } else if (robot.state == 6) {  // Advance a little then turn to place the box //// Problem?
      robot.solenoid_state = 1;
      //if (robot.rel_s < 0.1)
      //if (robot.rel_s < 0.1) robot.followLineRight(IRLine, 0.1, -0.04);
      //else if (robot.rel_s < 0.33) robot.followLineLeft(IRLine, 0.1, -0.04);
      //else 
      robot.setRobotVW(0.0, -1); // driving back
      
    } else if (robot.state == 7) {  
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.05); 

    } else if (robot.state == 8) { // Drop the box and go back
      robot.solenoid_state = 0;
      //if (robot.tis < 2000) robot.setRobotVW(-0.1, 0);
      //else robot.setRobotVW(0, 0);
       robot.setRobotVW(-0.1, 0);

    } else if (robot.state == 9) {  // Turn 180 degrees
      robot.solenoid_state = 0;
      //robot.setRobotVW(0,2.5);
      robot.setRobotVW(0, -1.25); // 1.25
      
    } else if (robot.state == 10) {  // turn left
      robot.solenoid_state = 0;
      robot.followLineLeft(IRLine, 0.1, -0.05);

    } else if (robot.state == 11) {  // lon travel and turn right 
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.05);
/*
    } else if (robot.state == 12) {  // Go: Get second box
      robot.solenoid_state = 1;
      //robot.followLineLeft(IRLine, 0.01, -0.05);
      //robot.followLineLeft(IRLine, 0.0, -0.3); // speed, turning 
      robot.setRobotVW(0, 2.5); // 1.25 */
      
    } else if (robot.state == 12) {  // Go: Get second box
      robot.solenoid_state = 1;    
      //robot.followLineLeft(IRLine, 0.06, -0.022); // speed, turning    
      
      robot.setRobotVW(0.1, 1.25);

      /*if (robot.tis < 10000) 
        robot.followLineLeft(IRLine, 0.2, -0.3); //0.06, -0.05); // -0.022 // 4000 vorheriger TH // 0.06 -0.022
      else 
        robot.followLineLeft(IRLine, 0.2, -0.05); // -0.022
      //else robot.setRobotVW(0.1, 0);*/
     
     } else if (robot.state == 13) {
      robot.solenoid_state = 1;    
      robot.followLineLeft(IRLine, 0.1, -0.05); // speed, turning    //1.25 

    } else if (robot.state == 14) {
      robot.solenoid_state = 1;    
      robot.setRobotVW(0.2, 0); //driving forward 0.1


    } else if (robot.state == 15) {  // Go back with the box
      robot.solenoid_state = 1;
      //if (robot.tis < 900) 
      robot.setRobotVW(-0.1, 0); //////// -0.2 

    } else if (robot.state == 16) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      robot.setRobotVW(0, 1.25); // turning clockwise 0.1 // minus other direction of turning
    
    } else if (robot.state == 17) {  // long travel to the box final destination
      robot.solenoid_state = 1;
      //followLineRight(80 - 40 * (IRLine.crosses >= 5), -2.0);
      robot.followLineRight(IRLine, 0.1, -0.05);

     } else if (robot.state == 18) {
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.1, -0.05);
     
     } else if (robot.state == 19) {
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.05);
     
     } else if (robot.state == 20) {
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.1, -0.05);

     } else if (robot.state == 21) {
      robot.solenoid_state = 0; /////// 0 
      robot.followLineRight(IRLine, 0.1, -0.05);


     } else if (robot.state == 22) {
      //robot.solenoid_state = 1;
      //robot.followLineRight(IRLine, 0.1, -0.05);
      robot.solenoid_state = 0;
       robot.setRobotVW(-0.1, 0);
/*
     } else if (robot.state == 22) {
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.1, -0.05);

     } else if (robot.state == 23) {
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.05, -0.1);  // ?  
      //robot.followLineRight(IRLine, 0.1, -0.022);
      //if (robot.tis < 9000) robot.followLineRight(IRLine, 0.1, -0.022); // -0.022 // 4000 vorheriger TH
      //else robot.followLineRight(IRLine, 0.15, -0.015); // -0.022

     } else if (robot.state == 24) { // drive backwards
      robot.solenoid_state = 0;
      // tis will never be set on 0 again, so its just counting the whole time?
      //if (robot.tis < 4000) 
      robot.setRobotVW(-0.2, 0); 
      //else robot.setRobotVW(0, 0); */

    /*} else if (robot.state == 10) { //Test
      robot.setRobotVW(0.1, 0);    */  

    } else if (robot.state == 100) {
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (robot.state == 101) {
      //robot.v_req = 0;
      //robot.w_req = 0;

    } else if (robot.state == 199) {
      /*robot.v_req = 0.1;
      robot.w_req = 4 * IRLine.IR_values[4] / 1024.0 
                  + 2 * IRLine.IR_values[3] / 1024.0
                  - 2 * IRLine.IR_values[1] / 1024.0
                  - 4 * IRLine.IR_values[0] / 1024.0;*/

    } else if (robot.state == 200) {
      robot.PWM_1 = 0;
      robot.PWM_2 = 0;

    } else if (robot.state == 201) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;
    
    } else if (robot.state == 202) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;      
    }
  
}
