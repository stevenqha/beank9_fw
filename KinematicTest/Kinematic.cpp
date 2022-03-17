#include <cstdio>
#include <cmath>
#include "Kinematic.h"

#define SHIN_LENGTH 150 //mm
#define THIGH_LENGTH 150 // mm
#define HIP_OFFSET_FR 102.46f // mm
#define HIP_OFFSET_FL 102.434f  // mm
#define BODY_WIDTH 64.5f // mm
#define BODY_LENGTH 255.0465f // mm

void kinematic(leg_e leg, float x, float y, float z, float roll, float pitch, float yaw) {
    printf("Leg: ");
    if (leg == LEG_FR) {
        printf(" LEG_FR\n");
    }
    else if (leg == LEG_FL) {
        printf(" LEG_FL\n");
    }
    else if (leg == LEG_BR) {
        printf(" LEG_BR\n");
    }
    else if (leg == LEG_BL) {
        printf(" LEG_BL\n");
    }

    // New
    // *** PITCH AXIS ***
    // Positive pitch is front of robot tilting counter clockwise
    pitch = pitch * M_PI / 180; // Convert from degrees to radians

    if (leg == LEG_BR || leg == LEG_BL) {
        x *= -1;      // switch over x for each end of the robot
        pitch *= -1;  // invert pitch for the end of the robot
    }

    printf("xin: %.3f zin: %.3f pitch: %.3f\n", x, z, pitch);

    //calc top triangle sides
    float legDiffPitch = sin(pitch) * BODY_LENGTH;
    float bodyDiffPitch = cos(pitch) * BODY_LENGTH;
    printf("a: %.3f b: %.3f \n", bodyDiffPitch, legDiffPitch);

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


    printf("c: %.3f e: %.3f h: %.3f\n", legDiffPitch, footDisplacementPitch, leg_length);
    printf("phi1/2: %.3f phi3/4: %.3f\n", footDisplacementAnglePitch * 180 / M_PI, footWholeAnglePitch * 180 / M_PI);
    printf("z : %.3f x : %.3f \n\n", z, x);


    // *** ROLL AXIS ***

    //turn around roll angle for each side of the robot
    if (leg == LEG_FL || leg == LEG_BL) {
        y *= -1;
    }
    else if (leg == LEG_FR || leg == LEG_BR) {
        roll *= -1;
    }

    roll = (M_PI/180) * roll;       // convert roll angle to radians

    // calc the top triangle sides
    float legDiffRoll = sin(roll) * BODY_WIDTH;
    float bodyDiffRoll = cos(roll) * BODY_WIDTH;

    // calc actual height from the ground for each side
    legDiffRoll = z + legDiffRoll;

    // calc foot displacement
    float footDisplacementRoll = (BODY_WIDTH - bodyDiffRoll) + y;

    //calc smaller displacement angle
    float footDisplacementAngleRoll = atan2(footDisplacementRoll, legDiffRoll);

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    float zz1a = legDiffRoll/cos(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    float footWholeAngleRoll = footDisplacementAngleRoll - roll;

    //calc actual leg length - the new Z to pass on
    z = cos(footWholeAngleRoll) * zz1a;

    //calc new Y to pass on
    y = sin(footWholeAngleRoll) * zz1a;

    printf("z : %.3f y : %.3f \n\n", z, y);
}