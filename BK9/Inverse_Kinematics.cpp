#include "Inverse_Kinematics.h"
#include "BK9_Hardware.h"
#include <math.h>
#include "Arduino.h"

// Inverse kinematics based on James Bruton's minidog
void inv_kin(leg_t* leg, float x, float y, float z, float roll, float pitch, float yaw) {
    leg_e leg_num = leg->leg;
    
    // convert degrees to radians for the calcs
    yaw = (M_PI/180) * yaw;

    // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
    if (leg_num == LEG_FL) {         // front left leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR);
       x = x - BODY_LENGTH;
    }
    else if (leg_num == LEG_FR) {    // front right leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR);
       x = x - BODY_LENGTH;
    }
    else if (leg_num == LEG_BL) {    // back left leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x + BODY_LENGTH;
    }
    else if (leg_num == LEG_BR) {    // back right leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x + BODY_LENGTH;
    }

    //calc existing angle of leg from cetre
    float existingAngle = atan(y/x);

    // calc radius from centre
    float radius = y/sin(existingAngle);

    //calc demand yaw angle
    float demandYaw = existingAngle + yaw;

    // calc new X and Y based on demand yaw angle
    x = radius * cos(demandYaw);           // calc new X and Y based on new yaw angle
    y = radius * sin(demandYaw);

    // remove the offsets so we pivot around 0/0 x/y
    if (leg_num == LEG_FL) {         // front left leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR);
       x = x + BODY_LENGTH;
    }
    else if (leg_num == LEG_FR) {    // front right leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR);
       x = x + BODY_LENGTH;
    }
    else if (leg_num == LEG_BL) {    // back left leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR);
       x = x - BODY_LENGTH;
    }
    else if (leg_num == LEG_BR) {    // back right leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR);
       x = x - BODY_LENGTH;
    }

    // ----------------------------------------------------------------------------------------------------------
    // *** PITCH AXIS ***
    // Positive pitch is front of robot tilting counter clockwise
    pitch = (M_PI/180) * pitch;   // convert pitch to radians
    
    if (leg_num == LEG_BR || leg_num == LEG_BL) {
        x *= -1;      // switch over x for each end of the robot
        pitch *= -1;  // invert pitch for the end of the robot
    }
    
    //calc top triangle sides
    float legDiffPitch = sin(pitch) * BODY_LENGTH;
    float bodyDiffPitch = cos(pitch) * BODY_LENGTH;

    // calc actual height from the ground for each side
    legDiffPitch = z + legDiffPitch;

    // calc foot displacement
    float footDisplacementPitch = (BODY_LENGTH - bodyDiffPitch) - x;

    //calc smaller displacement angle
    float footDisplacementAnglePitch = atan2(footDisplacementPitch, legDiffPitch);

    // calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    float leg_length = legDiffPitch/cos(footDisplacementAnglePitch);

    // calc the whole angle for the leg
    float footWholeAnglePitch = footDisplacementAnglePitch - pitch;

    //calc actual leg length - the new Z to pass on
    z = cos(footWholeAnglePitch) * leg_length;

    //calc new Z to pass on
    x = sin(footWholeAnglePitch) * leg_length * -1;

//    Serial.print("Pitch z: ");
//    Serial.println(z);
//    Serial.print("Pitch x: ");
//    Serial.println(x);
    // ---------------------------------------------------------------------------------------------------------

    // *** NEW ROLL ***
    // Positive is CCW from the front of the robot
    
    // Adjust y and roll depending on the leg
    if (leg_num == LEG_FL || leg_num == LEG_BL) {
        y *= -1; // Moving the leg in negative y results in positive y for robot
    }
    else if (leg_num == LEG_FR || leg_num == LEG_BR) {
        roll *= -1; 
    }

    roll = (M_PI/180) * roll;       // convert roll angle to radians

    // calc the top triangle sides
    float legDiffRoll = sin(roll) * BODY_WIDTH;
    float bodyDiffRoll = cos(roll) * BODY_WIDTH;

    // calc actual height from the ground for each side
    legDiffRoll = z + legDiffRoll;

    // calc foot displacement
    float footDisplacementRoll = (BODY_WIDTH - bodyDiffRoll) + HIP_OFFSET_FR + y;

    //calc smaller displacement angle
    float footDisplacementAngleRoll = atan2(footDisplacementRoll, legDiffRoll);

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    float zz1a = legDiffRoll/cos(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    float footWholeAngleRoll = footDisplacementAngleRoll - roll;

    //calc actual leg length - the new Z to pass on
    z = cos(footWholeAngleRoll) * zz1a;

    //calc new Y to pass on
    y = sin(footWholeAngleRoll) * zz1a - HIP_OFFSET_FR;

//    Serial.print("Roll z: ");
//    Serial.println(z);
//    Serial.print("Roll y: ");
//    Serial.println(y);

    // *** TRANSLATION AXES ***
    // Y TRANSLATION (Side to side)
    // calculate the hip joint and new leg length based on how far the robot moves sideways

//    if (leg == LEG_FR || leg == LEG_BR) {
//        y *= -1;
//    }
    
    float length_y = y + HIP_OFFSET_FR;                  // calc sideways distance of foot from pivot point centre
    float hipAngle1a = atan(length_y/z);             // first angle (that is there at rest due to the offset)
    float hipHyp = length_y / (sin(hipAngle1a));       // hypotenuse from joint pivot to foot

    // second triangle
    float hipAngle1b = asin(HIP_OFFSET_FR/hipHyp) ;     // calc 'the other angle' in the triangle
    float hipAngle1 = (M_PI - (M_PI/2) - hipAngle1b) + hipAngle1a;     // calc total hip angle
    hipAngle1 = hipAngle1 - 1.570796;           // take away offset for rest position, hip is already at 90 degrees
    float hipAngle1Degrees = ((hipAngle1 * (180/M_PI)));   // convert to degrees and take off rest position
    
    // calc new leg length to give to the code  below
    z = HIP_OFFSET_FR / tan(hipAngle1b);           // new leg length

//    Serial.print("New z1 : ");
//    Serial.print(z);
//    Serial.print(" hip ang : ");
//    Serial.println(hipAngle1Degrees, 5);


    // X TRANSLATION (Forward/backward)
    // Invert command for back legs
//    if (leg == LEG_BR || leg == LEG_BL) {
//        x = -1 * x;    
//    }
    
    // Calculate shoulder angle and new z based on desired x
    float shoulder_ang2 = atan(x / z);
    z = z / cos(shoulder_ang2);
    shoulder_ang2 *= (180 / M_PI); // rad to degrees
//    Serial.print("New z2 : ");
//    Serial.print(z);
//    Serial.print(" ang2 : ");
//    Serial.println(shoulder_ang2, 5);

    // Z TRANSLATION (Up and down)
    // Calculate shoulder and knee angle based on desired height
    float num = (THIGH_LENGTH * THIGH_LENGTH) + (z*z) - (SHIN_LENGTH * SHIN_LENGTH);
    float den = 2 * THIGH_LENGTH * z;
    float quo = num / den;
    float shoulder_ang = acos(quo);
    float knee_ang = M_PI - (shoulder_ang*2);

    // Convert rad to degrees
    shoulder_ang = shoulder_ang * (180/M_PI);
    knee_ang = knee_ang * (180/M_PI);
    
//    Serial.print(shoulder_ang, 5);
//    Serial.print(", ");
//    Serial.println(knee_ang, 5);

//    Serial.print("total shoulder ang : ");
//    Serial.println(shoulder_ang2 + shoulder_ang, 5);

    int shoulder_sig, knee_sig, hip_sig;
    // Convert joint angle to servo angle based on gear ratio
    shoulder_ang = (shoulder_ang + shoulder_ang2) * GEAR_RATIO;
    knee_ang = (135 - knee_ang) * GEAR_RATIO;
    hipAngle1Degrees = hipAngle1Degrees * GEAR_RATIO;

    // Front Right Leg
    if (leg_num == LEG_FR) { 
        // Convert to microseconds for servo
        shoulder_sig = MIN_SIG + shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MIN_SIG + knee_ang * DEG_TO_US - 10;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
        hip_sig = FR_HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
//        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);
    }
    else if (leg_num == LEG_FL) {
        // Convert to microseconds for servo
        shoulder_sig = MAX_SIG - shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MAX_SIG - knee_ang * DEG_TO_US + 50;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
//        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = FL_HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

//        fl_shoulder.writeMicroseconds(shoulder_sig);
//        fl_knee.writeMicroseconds(knee_sig);
//        fl_hip.writeMicroseconds(hip_sig);
    }
    else if (leg_num == LEG_BR) {
        // Convert to microseconds for servo
        shoulder_sig = MAX_SIG - shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MAX_SIG - knee_ang * DEG_TO_US + 60;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
        hip_sig = BR_HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
//        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

//        br_shoulder.writeMicroseconds(shoulder_sig);
//        br_knee.writeMicroseconds(knee_sig);
//        br_hip.writeMicroseconds(hip_sig);
    }
    else { // LEG_BL
        // Convert to microseconds for servo
        shoulder_sig = MIN_SIG + shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MIN_SIG + knee_ang * DEG_TO_US - 50;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
//        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = BL_HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

//        bl_shoulder.writeMicroseconds(shoulder_sig);
//        bl_knee.writeMicroseconds(knee_sig);
//        bl_hip.writeMicroseconds(hip_sig);
    }

    leg->shoulder.writeMicroseconds(shoulder_sig);
    leg->knee.writeMicroseconds(knee_sig);
    leg->hip.writeMicroseconds(hip_sig);
    
//    Serial.print(shoulder_sig);
//    Serial.print(", ");
//    Serial.print(knee_sig);
//    Serial.print(", ");
//    Serial.println(hip_sig);
}

//struct angles {
//  double tetta;
//  double alpha;
//  double gamma;
//};
//
//// From https://github.com/miguelasd688/4-legged-robot-model
//// Much faster need to test out though
//struct angles legFR(double x4, double y4, double z4) {
//#define L1 HIP_OFFSET_FR
//#define L2 THIGH_LENGTH
//#define L3 SHIN_LENGTH
//#define MAX_GAMMA 50
//
//  struct angles ang;
//  double D;
//
//  D = (sq(x4) + sq(-y4) - sq(L1) + sq(z4) - sq(L2) - sq(L3)) / (2 * L2 * L3);
//  //  if (D >= 1){D=1;}
//  //  else if (D <= 0){D=0;}
//  /////////////////////////////////////////////DOMINIO
//  ang.tetta = -atan2(y4, x4) - atan2(sqrt(sq(x4) + sq(-y4) - sq(L1)), -L1);
//  ang.gamma = atan2(sqrt(1 - sq(D)), D);
//  ang.alpha = atan2(z4, sqrt(sq(x4) + sq(-y4) - sq(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
//  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
//  ang.alpha = -ang.alpha * 360 / (2 * PI);
//  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;
//
//  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
//  if (ang.gamma >= MAX_GAMMA) {
//    ang.gamma = MAX_GAMMA;
//  }
//  return ang;
//}
