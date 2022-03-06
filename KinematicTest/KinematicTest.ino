#include <Servo.h>

int initStart;
int state;
float rate;

int targetLeg1x;
int targetLeg1z;
int prevLeg1x;
int prevLeg1z;
float currentLeg1x;
float currentLeg1z;
float stepDiffLeg1x;
float stepDiffLeg1z;

int targetLeg2x;
int targetLeg2z;
int prevLeg2x;
int prevLeg2z;
float currentLeg2x;
float currentLeg2z;
float stepDiffLeg2x;
float stepDiffLeg2z;

int targetLeg3x;
int targetLeg3z;
int prevLeg3x;
int prevLeg3z;
float currentLeg3x;
float currentLeg3z;
float stepDiffLeg3x;
float stepDiffLeg3z;

int targetLeg4x;
int targetLeg4z;
int prevLeg4x;
int prevLeg4z;
float currentLeg4x;
float currentLeg4z;
float stepDiffLeg4x;
float stepDiffLeg4z;

int targetLeg1y;
int prevLeg1y;
float currentLeg1y;
float stepDiffLeg1y;

int walkXPos1;
int walkXPos2;
int walkXPos3; 
int walkXPos4; 
int walkXPos5;
int walkXPos6; 
int walkXPos7; 
int walkXPos8; 

int walkYPos1;
int walkYPos2;
int walkYPos3;
int walkYPos4;
int walkYPos5;
int walkYPos6;
int walkYPos7;
int walkYPos8;

int walkZPos1;
int walkZPos2;
int walkZPos3;
int walkZPos4;
int walkZPos5;
int walkZPos6;
int walkZPos7;
int walkZPos8;

Servo fr_hip;
Servo fr_shoulder;
Servo fr_knee;

Servo fl_hip;
Servo fl_shoulder;
Servo fl_knee;

Servo br_hip;
Servo br_shoulder;
Servo br_knee;

Servo bl_hip;
Servo bl_shoulder;
Servo bl_knee;


#define FR_HIP_PIN 22
#define FR_SHOULDER_PIN 23
#define FR_KNEE_PIN 0

#define FL_HIP_PIN 9
#define FL_SHOULDER_PIN 8
#define FL_KNEE_PIN 7

#define BR_HIP_PIN 15
#define BR_SHOULDER_PIN 14
#define BR_KNEE_PIN 13

#define BL_HIP_PIN 10
#define BL_SHOULDER_PIN 11
#define BL_KNEE_PIN 12

#define MIN_SIG 500
#define MAX_SIG 2500

#define BUF_LENGTH 10

#define SHIN_LENGTH 150 //mm
#define THIGH_LENGTH 150 // mm
#define HIP_OFFSET_FR 102.46f // mm
#define HIP_OFFSET_FL 102.434f  // mm
#define BODY_WIDTH 64.5f // mm
#define BODY_LENGTH 255.0465f // mm


#define GEAR_RATIO 3
#define DEG_TO_US 7.407407f
#define SERVO_SIG_OFFSET 500
#define HIP_SIG_OFFSET 1500

typedef enum {
    LEG_FR,
    LEG_FL,
    LEG_BR,
    LEG_BL
} leg_e;

void kinematic(leg_e leg, float x, float y, float z, float roll, float pitch, float yaw) {
    
    // convert degrees to radians for the calcs
    yaw = (PI/180) * yaw;

    // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
    if (leg == LEG_FL) {         // front left leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x - BODY_LENGTH;      
    }
    else if (leg == LEG_FR) {    // front right leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR);
       x = x - BODY_LENGTH; 
    }
    else if (leg == LEG_BL) {    // back left leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x + BODY_LENGTH;
    }
    else if (leg == LEG_BR) {    // back right leg
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
    if (leg == LEG_FL) {         // front left leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x + BODY_LENGTH;      
    }
    else if (leg == LEG_FR) {    // front right leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR);
       x = x + BODY_LENGTH; 
    }
    else if (leg == LEG_BL) {    // back left leg
       y = y + (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x - BODY_LENGTH;
    }
    else if (leg == LEG_BR) {    // back right leg
       y = y - (BODY_WIDTH+HIP_OFFSET_FR); 
       x = x - BODY_LENGTH;
    }

    // *** PITCH AXIS ***

    //turn around the pitch for front or back of the robot
    if (leg == LEG_FR || leg == LEG_FL) {
      pitch = 0-pitch;      
    }
    else if (leg == LEG_BR || leg == LEG_BL) {
      x = x*-1;       // switch over x for each end of the robot
    }

    // convert pitch to degrees
    pitch = (PI/180) * pitch;

    //calc top triangle sides
    float legDiffPitch = sin(pitch) * BODY_LENGTH;
    float bodyDiffPitch = cos(pitch) * BODY_LENGTH;

    // calc actual height from the ground for each side
    legDiffPitch = z - legDiffPitch;

    // calc foot displacement
    float footDisplacementPitch = ((bodyDiffPitch - BODY_LENGTH)*-1)+x;

    //calc smaller displacement angle
    float footDisplacementAnglePitch = atan(footDisplacementPitch/legDiffPitch);

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    float zz2a = legDiffPitch/cos(footDisplacementAnglePitch);

    // calc the whole angle for the leg
    float footWholeAnglePitch = footDisplacementAnglePitch + pitch;

    //calc actual leg length - the new Z to pass on
    z = cos(footWholeAnglePitch) * zz2a;

    //calc new Z to pass on
    x = sin(footWholeAnglePitch) * zz2a;

//    if (leg == LEG_BR || leg == LEG_BL ){     // switch back X for the back of the robot
//      x = x *-1;
//    }

    // *** ROLL AXIS ***

    //turn around roll angle for each side of the robot
    if (leg == LEG_FL || leg == LEG_BL) {
      y = y*-1;
    }
    else if (leg == LEG_FR || leg == LEG_BR) {
      roll = 0-roll;
    }   

    // convert roll angle to radians
    float rollAngle = (PI/180) * roll;    //convert degrees from the stick to radians

    // calc the top triangle sides
    float legDiffRoll = sin(rollAngle) * BODY_WIDTH;
    float bodyDiffRoll = cos(rollAngle) * BODY_WIDTH;
    
    // calc actual height from the ground for each side
    legDiffRoll = z - legDiffRoll;              

    // calc foot displacement
    float footDisplacementRoll = ((bodyDiffRoll - BODY_WIDTH)*-1)-y;

    //calc smaller displacement angle
    float footDisplacementAngleRoll = atan(footDisplacementRoll/legDiffRoll);  

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    float zz1a = legDiffRoll/cos(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    float footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;

    //calc actual leg length - the new Z to pass on
    z = cos(footWholeAngleRoll) * zz1a;

    //calc new Y to pass on
    y = sin(footWholeAngleRoll) * zz1a; 

    // *** TRANSLATION AXES ***
    // Y TRANSLATION (Side to side)
    // calculate the hip joint and new leg length based on how far the robot moves sideways
    // first triangle
//    float hipAngle1 = atan(y/z);    
//    float hipAngle1Degrees = ((hipAngle1 * (180/PI)));   // convert to degrees and take off rest position
//    z = z/cos(hipAngle1);

//    if (leg == LEG_FR || leg == LEG_BR) {
//        y *= -1;
//    }
    
    float length_y = y + HIP_OFFSET_FR;                  // calc sideways distance of foot from pivot point centre
    float hipAngle1a = atan(length_y/z);             // first angle (that is there at rest due to the offset)
    float hipHyp = length_y / (sin(hipAngle1a));       // hypotenuse from joint pivot to foot

    // second triangle
    float hipAngle1b = asin(HIP_OFFSET_FR/hipHyp) ;     // calc 'the other angle' in the triangle
    float hipAngle1 = (PI - (PI/2) - hipAngle1b) + hipAngle1a;     // calc total hip angle
    hipAngle1 = hipAngle1 - 1.570796;           // take away offset for rest position, hip is already at 90 degrees
    float hipAngle1Degrees = ((hipAngle1 * (180/PI)));   // convert to degrees and take off rest position
    
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
    shoulder_ang2 *= (180 / PI); // rad to degrees
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
    float knee_ang = PI - (shoulder_ang*2);

    // Convert rad to degrees
    shoulder_ang = shoulder_ang * (180/PI);
    knee_ang = knee_ang * (180/PI);
    
//    Serial.print(shoulder_ang, 5);
//    Serial.print(", ");
//    Serial.println(knee_ang, 5);

    int shoulder_sig, knee_sig, hip_sig;
    // Convert joint angle to servo angle based on gear ratio
    shoulder_ang = (shoulder_ang + shoulder_ang2) * GEAR_RATIO;
    knee_ang = (135 - knee_ang) * GEAR_RATIO;
    hipAngle1Degrees = hipAngle1Degrees * GEAR_RATIO;

    // Front Right Leg
    if (leg == LEG_FR) { 
        // Convert to microseconds for servo
        shoulder_sig = MIN_SIG + shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MIN_SIG + knee_ang * DEG_TO_US;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
//        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

        fr_shoulder.writeMicroseconds(shoulder_sig);
        fr_knee.writeMicroseconds(knee_sig);
        fr_hip.writeMicroseconds(hip_sig);
    }
    else if (leg == LEG_FL) {
        // Convert to microseconds for servo
        shoulder_sig = MAX_SIG - shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MAX_SIG - knee_ang * DEG_TO_US;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
//        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

        fl_shoulder.writeMicroseconds(shoulder_sig);
        fl_knee.writeMicroseconds(knee_sig);
        fl_hip.writeMicroseconds(hip_sig);
    }
    else if (leg == LEG_BR) {
        // Convert to microseconds for servo
        shoulder_sig = MAX_SIG - shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MAX_SIG - knee_ang * DEG_TO_US;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
//        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

        br_shoulder.writeMicroseconds(shoulder_sig);
        br_knee.writeMicroseconds(knee_sig);
        br_hip.writeMicroseconds(hip_sig);
    }
    else { // LEG_BL
        // Convert to microseconds for servo
        shoulder_sig = MIN_SIG + shoulder_ang * DEG_TO_US;
        shoulder_sig = constrain(shoulder_sig, MIN_SIG, MAX_SIG);
        
        knee_sig = MIN_SIG + knee_ang * DEG_TO_US;
        knee_sig = constrain(knee_sig, MIN_SIG, MAX_SIG);
    
//        hip_sig = HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
        hip_sig = HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
        hip_sig = constrain(hip_sig, MIN_SIG, MAX_SIG);

        bl_shoulder.writeMicroseconds(shoulder_sig);
        bl_knee.writeMicroseconds(knee_sig);
        bl_hip.writeMicroseconds(hip_sig);
    }
    
//    Serial.print(shoulder_sig);
//    Serial.print(", ");
//    Serial.print(knee_sig);
//    Serial.print(", ");
//    Serial.println(hip_sig);
}

char buffer[BUF_LENGTH];
int numBytes = 0;


int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

struct angles {
  double tetta;
  double alpha;
  double gamma;
};

struct angles legFR(double x4, double y4, double z4) {
#define L1 40
#define L2 100
#define L3 100
#define MAX_GAMMA 50

  struct angles ang;
  double D;

  D = (sq(x4) + sq(-y4) - sq(L1) + sq(z4) - sq(L2) - sq(L3)) / (2 * L2 * L3);
  //  if (D >= 1){D=1;}
  //  else if (D <= 0){D=0;}
  /////////////////////////////////////////////DOMINIO
  ang.tetta = -atan2(y4, x4) - atan2(sqrt(sq(x4) + sq(-y4) - sq(L1)), -L1);
  ang.gamma = atan2(sqrt(1 - sq(D)), D);
  ang.alpha = atan2(z4, sqrt(sq(x4) + sq(-y4) - sq(L1))) - atan2(L3 * sin(ang.gamma), L2 + L3 * cos(ang.gamma));
  ang.tetta = ang.tetta * 360 / (2 * PI) + 270;
  ang.alpha = -ang.alpha * 360 / (2 * PI);
  ang.gamma = ang.gamma * 360 / (2 * PI) - 90;

  //  Serial.print("\t");Serial.print(ang.tetta);Serial.print(" - ");Serial.print(ang.alpha);Serial.print(" - ");Serial.println(ang.gamma);
  if (ang.gamma >= MAX_GAMMA) {
    ang.gamma = MAX_GAMMA;
  }
  return ang;
}

void reset_servos() {
    fr_hip.writeMicroseconds(1500);
    fr_shoulder.writeMicroseconds(2500);
    fr_knee.writeMicroseconds(1500);

    fl_hip.writeMicroseconds(1500);
    fl_shoulder.writeMicroseconds(500);
    fl_knee.writeMicroseconds(1500);

    br_hip.writeMicroseconds(1500);
    br_shoulder.writeMicroseconds(500);
    br_knee.writeMicroseconds(1500);

    bl_hip.writeMicroseconds(1500);
    bl_shoulder.writeMicroseconds(2500);
    bl_knee.writeMicroseconds(1500);
}

#define RESET 1
#define Z_TEST 0
#define X_TEST 0
#define Y_TEST 0
#define STABLE_WALK 0

int desired_z = 200; // 90 degree between joints
int desired_x = 0;
int desired_y = 0;
int desired_pitch = 0;
int desired_roll = 0;
int desired_yaw = 0;

void setup() {
    fr_hip.attach(FR_HIP_PIN, MIN_SIG, MAX_SIG);
    fr_shoulder.attach(FR_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    fr_knee.attach(FR_KNEE_PIN, MIN_SIG, MAX_SIG);

    fl_hip.attach(FL_HIP_PIN, MIN_SIG, MAX_SIG);
    fl_shoulder.attach(FL_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    fl_knee.attach(FL_KNEE_PIN, MIN_SIG, MAX_SIG);

    br_hip.attach(BR_HIP_PIN, MIN_SIG, MAX_SIG);
    br_shoulder.attach(BR_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    br_knee.attach(BR_KNEE_PIN, MIN_SIG, MAX_SIG);

    bl_hip.attach(BL_HIP_PIN, MIN_SIG, MAX_SIG);
    bl_shoulder.attach(BL_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    bl_knee.attach(BL_KNEE_PIN, MIN_SIG, MAX_SIG);
    
    Serial.begin(115200);
    while(!Serial) {}

    #if RESET
    reset_servos();
    #else

    elapsedMicros time_e = 0;
    kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
    Serial.println(time_e);

    time_e = 0;
    kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
    Serial.println(time_e);

    time_e = 0;
    kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
    Serial.println(time_e);

    time_e = 0;
    kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
    Serial.println(time_e);

    time_e = 0;
    struct angles a = legFR(10, 10, 10);
    Serial.println(time_e);
    #endif // RESET
    
    delay(1000);
}

void loop() {
    numBytes = Serial.available();
    if(readline(Serial.read(), buffer, BUF_LENGTH) > 0) {
//        desired_z = atoi(buffer);
//        Serial.println(desired_z);
//        desired_x = atoi(buffer);
//        Serial.println(desired_x);
//        desired_y = atoi(buffer);
//        Serial.println(desired_y);
//        desired_roll = atoi(buffer);
//        Serial.println(desired_roll);
//        desired_pitch = atoi(buffer);
//        Serial.println(desired_pitch);
//        desired_yaw = atoi(buffer);
//        Serial.println(desired_yaw);
        
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
    }

#if Z_TEST // Z-TEST
    for (int i = 200; i >= 114; --i) {
        desired_z--;
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        delay(20);    
    }

    for (int i = 114; i <= 200; ++i) {
        desired_z++;
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        delay(20);    
    }
#elif X_TEST || Y_TEST

    // 0 - 50
    for (int i = 0; i <= 50; ++i) {
        #if X_TEST
        desired_x++;
        #elif Y_TEST
        desired_y++;
        #endif
        
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        delay(45);    
    }

    // 50 to -50
    for (int i = 0; i <= 100; ++i) {
        #if X_TEST
        desired_x--;
        #elif Y_TEST
        desired_y--;
        #endif
        
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        delay(45);    
    }

    // -50 - 0
    for (int i = 0; i <= 50; ++i) {
        #if X_TEST
        desired_x++;
        #elif Y_TEST
        desired_y++;
        #endif
        
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        delay(45);    
    }
#endif
    
}
