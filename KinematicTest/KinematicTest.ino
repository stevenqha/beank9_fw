#include <Servo.h>
#include <Ramp.h>

int initStart = 0;
int state = 0;
float rate = 10;

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
#define FR_KNEE_PIN 1

#define FL_HIP_PIN 0
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
#define FR_HIP_SIG_OFFSET 1610
#define FL_HIP_SIG_OFFSET 1400
#define BR_HIP_SIG_OFFSET 1460
#define BL_HIP_SIG_OFFSET 1630

#define DEFAULT_Z 200 // mm (results in 90 degrees between thigh and shin)
#define DEFAULT_Y_OFFSET 10 // mm

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

//    Serial.print("total shoulder ang : ");
//    Serial.println(shoulder_ang2 + shoulder_ang, 5);

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
    
        hip_sig = FR_HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
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
        hip_sig = FL_HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
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
    
        hip_sig = BR_HIP_SIG_OFFSET - hipAngle1Degrees * DEG_TO_US;
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
        hip_sig = BL_HIP_SIG_OFFSET + hipAngle1Degrees * DEG_TO_US;
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
#define L1 HIP_OFFSET_FR
#define L2 THIGH_LENGTH
#define L3 SHIN_LENGTH
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

void rest_position() {
//    kinematic(LEG_FR, -43.5, -DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    kinematic(LEG_FL, -43.5, DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    kinematic(LEG_BR, 43.5, -DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    kinematic(LEG_BL, 43.5, DEFAULT_Y_OFFSET, 115, 0, 0, 0);

    kinematic(LEG_FR, -43.5, 0, 115, 0, 0, 0);
    kinematic(LEG_FL, -43.5, 0, 115, 0, 0, 0);
    kinematic(LEG_BR, 43.5, 0, 115, 0, 0, 0);
    kinematic(LEG_BL, 43.5, 0, 115, 0, 0, 0);
    
    delay(1000);
}

void stand() {
    rampInt z_ramp;
    rampFloat x_ramp;

    z_ramp.go(115);
    x_ramp.go(43.5);

    z_ramp.go(200, 2000, LINEAR, ONCEFORWARD);
    x_ramp.go(0, 2000, LINEAR, ONCEFORWARD);

    float z_val = 115;
    float x_val = 43.5;

    for (int i = 0; i < 200; i++) {
        z_val = z_ramp.update();
        x_val = x_ramp.update();
        
        kinematic(LEG_FR, -x_val, -DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        kinematic(LEG_FL, -x_val, DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        kinematic(LEG_BR, x_val, -DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        kinematic(LEG_BL, x_val, DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        
        delay(10);
    }
}

void stop_test() {
    
}

void x_test() {
    
}

#define RESET 0
#define Z_TEST 0
#define X_TEST 0
#define Y_TEST 0
#define STABLE_WALK 1

int desired_z = DEFAULT_Z; // 90 degree between joints
int desired_x = 0;
int desired_y = 0;
float desired_pitch = 0;
float desired_roll = 0;
float desired_yaw = 0;

unsigned long currentMillis;
unsigned long previousMillis;
bool toggle_walk = false;
bool incr = true;


rampInt test_ramp;
int test_mode = -1;

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
    //reset_servos();
    #else
    rest_position();

    // Wait for command from Jetson to stand
    while(!Serial.available());

    while(Serial.available() > 0) {
        char t = Serial.read();
    }
    
    stand();
//    elapsedMicros time_e = 0;
//    kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
//    Serial.println(time_e);
//
//    time_e = 0;
//    kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
//    Serial.println(time_e);
//
//    time_e = 0;
//    kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
//    Serial.println(time_e);
//
//    time_e = 0;
//    kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
//    Serial.println(time_e);
//
//    time_e = 0;
//    struct angles a = legFR(10, 10, 10);
//    Serial.println(time_e);
    #endif // RESET
    
    delay(1000);
}

void loop() {
    numBytes = Serial.available();
    if(readline(Serial.read(), buffer, BUF_LENGTH) > 0) {
        #if !STABLE_WALK
//        desired_z = atoi(buffer);
//        Serial.println(desired_z);
//        desired_x = atoi(buffer);
//        Serial.println(desired_x);
//        desired_y = atoi(buffer);
//        Serial.println(desired_y);
//        desired_roll = atoi(buffer);
//        Serial.println(desired_roll);
        desired_pitch = atoi(buffer);
        Serial.println(desired_pitch);
//        desired_yaw = atoi(buffer);
//        Serial.println(desired_yaw);
        kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);

        #else
        if (buffer[0] == 'x' || buffer[0] == 'y' || buffer[0] == 'z' || 
            buffer[0] == 'r' || buffer[0] == 'p' || buffer[0] == 'h' ||
            buffer[0] == 'w' || buffer[0] == 's' || buffer[0] == ' ' ||
            buffer[0] == 'c') {
            test_mode = buffer[0];

            char c = buffer[0];

            Serial.println(c);
        }
//        int val = atoi(buffer);
//
//        if (val > 1) {
//            rate = val;
//            Serial.println(rate);
//        }
//        else {
//            toggle_walk = val;
//        }
        #endif // STABLE_WALK
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
#elif STABLE_WALK
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {  // start timed event
          
        previousMillis = currentMillis;

        // define walking positions
        
//        walkXPos1 = -30;
//        walkXPos2 = 20;
//        walkXPos3 = 30; 
//        walkXPos4 = 20; 
//        walkXPos5 = 10;
//        walkXPos6 = 0; 
//        walkXPos7 = -10; 
//        walkXPos8 = -20;

        walkXPos1 = -60;
        walkXPos2 = 40;
        walkXPos3 = 60; 
        walkXPos4 = 40; 
        walkXPos5 = 20;
        walkXPos6 = 0; 
        walkXPos7 = -20; 
        walkXPos8 = -40;
        
        walkZPos1 = 215;    // leg down
        walkZPos2 = 180; //150;    // leg up
        walkZPos3 = 215;    // leg down
        walkZPos4 = 215;    // leg down
        walkZPos5 = 215;    // leg down
        walkZPos6 = 215;    // leg down
        walkZPos7 = 215;    // leg down
        walkZPos8 = 215;    // leg down
        
//        walkYPos1 = -25;
//        walkYPos2 = -25;
//        walkYPos3 = -25; 
//        walkYPos4 = -25;
//        walkYPos5 = 25;
//        walkYPos6 = 25;
//        walkYPos7 = 25;
//        walkYPos8 = 25;

        walkYPos1 = 0; //-25;
        walkYPos2 = 0; //-25;
        walkYPos3 = 0; //-25; 
        walkYPos4 = 0; //-25;
        walkYPos5 = 0; //25;
        walkYPos6 = 0; //25;
        walkYPos7 = 0; //25;
        walkYPos8 = 0; //25;


        if (test_mode == 'x') {
            if (incr) {
                desired_x++;

                if (desired_x >= 50) {
                    incr = false;    
                }
            }
            else {
                desired_x--;

                if (desired_x <= -50) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'y') {
            if (incr) {
                desired_y++;

                if (desired_y >= 50) {
                    incr = false;    
                }
            }
            else {
                desired_y--;

                if (desired_y <= -50) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'z') {
            if (incr) {
                desired_z++;

                if (desired_z >= DEFAULT_Z) {
                    incr = false;    
                }
            }
            else {
                desired_z--;

                if (desired_z <= 114) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'r') {
            if (incr) {
                desired_roll += 0.15;

                if (desired_roll >= 10) {
                    incr = false;    
                }
            }
            else {
                desired_roll -= 0.15;

                if (desired_roll <= -10) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'p') {
            if (incr) {
                desired_pitch+= 0.15;

                if (desired_pitch >= 8) {
                    incr = false;    
                }
            }
            else {
                desired_pitch-=0.15;

                if (desired_pitch <= -8) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'h') {
            if (incr) {
                desired_yaw+=0.15;

                if (desired_yaw >= 10) {
                    incr = false;    
                }
            }
            else {
                desired_yaw-=0.15;

                if (desired_yaw <= -10) {
                    incr = true;    
                }
            }

            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(35);
        }
        else if (test_mode == 'w') {
            if (initStart == 0) {
                currentLeg1x = walkXPos3;                        
                currentLeg1z = walkZPos3;                        // leg1 down
            
                currentLeg2x = walkXPos7;                        
                currentLeg2z = walkZPos7;                        // leg2 down
            
                currentLeg3x = walkXPos5;                        
                currentLeg3z = walkZPos5;                        // leg2 down
            
                currentLeg4x = walkXPos1;                        
                currentLeg4z = walkZPos1;                        // leg2 down
            
                currentLeg1y = walkYPos1;                        // lean
            
                initStart = 1;
            }
    
//            if (toggle_walk == true) {           // start state machine for walking
                if (state == 0) {
                    targetLeg1x = walkXPos3;
                    targetLeg1z = walkZPos3;
                    targetLeg2x = walkXPos7;
                    targetLeg2z = walkZPos7;
                    targetLeg3x = walkXPos5;
                    targetLeg3z = walkZPos5;
                    targetLeg4x = walkXPos1;
                    targetLeg4z = walkZPos1;
                    targetLeg1y = walkYPos1;                      
                    if (currentLeg1x >= targetLeg1x) {
                        state = 1;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 1) {  // LEG_BL Lifting
    //                desired_pitch = -1.5; 
    //                desired_roll = 1.5;
                    
                    targetLeg1x = walkXPos4;
                    targetLeg1z = walkZPos4;
                    targetLeg2x = walkXPos8;
                    targetLeg2z = walkZPos8;
                    targetLeg3x = walkXPos6;
                    targetLeg3z = walkZPos6;
                    targetLeg4x = walkXPos2;
                    targetLeg4z = walkZPos2;
                    targetLeg1y = walkYPos2; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 2;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 2) {
                    targetLeg1x = walkXPos5;
                    targetLeg1z = walkZPos5;
                    targetLeg2x = walkXPos1;
                    targetLeg2z = walkZPos1;
                    targetLeg3x = walkXPos7;
                    targetLeg3z = walkZPos7;
                    targetLeg4x = walkXPos3;
                    targetLeg4z = walkZPos3;
                    targetLeg1y = walkYPos3; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 3;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 3) {
                    targetLeg1x = walkXPos6;
                    targetLeg1z = walkZPos6;
                    targetLeg2x = walkXPos2;
                    targetLeg2z = walkZPos2;
                    targetLeg3x = walkXPos8;
                    targetLeg3z = walkZPos8;
                    targetLeg4x = walkXPos4;
                    targetLeg4z = walkZPos4;
                    targetLeg1y = walkYPos4; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 4;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 4) {
                    targetLeg1x = walkXPos7;
                    targetLeg1z = walkZPos7;
                    targetLeg2x = walkXPos3;
                    targetLeg2z = walkZPos3;
                    targetLeg3x = walkXPos1;
                    targetLeg3z = walkZPos1;
                    targetLeg4x = walkXPos5;
                    targetLeg4z = walkZPos5;
                    targetLeg1y = walkYPos5; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 5;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 5) {
                    targetLeg1x = walkXPos8;
                    targetLeg1z = walkZPos8;
                    targetLeg2x = walkXPos4;
                    targetLeg2z = walkZPos4;
                    targetLeg3x = walkXPos2;
                    targetLeg3z = walkZPos2;
                    targetLeg4x = walkXPos6;
                    targetLeg4z = walkZPos6;
                    targetLeg1y = walkYPos6; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 6;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 6) {
                    targetLeg1x = walkXPos1;
                    targetLeg1z = walkZPos1;
                    targetLeg2x = walkXPos5;
                    targetLeg2z = walkZPos5;
                    targetLeg3x = walkXPos3;
                    targetLeg3z = walkZPos3;
                    targetLeg4x = walkXPos7;
                    targetLeg4z = walkZPos7;
                    targetLeg1y = walkYPos7; 
                    if (currentLeg1x <= targetLeg1x) {
                        state = 7;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
            
                else if (state == 7) {
                    targetLeg1x = walkXPos2;
                    targetLeg1z = walkZPos2;
                    targetLeg2x = walkXPos6;
                    targetLeg2z = walkZPos6;
                    targetLeg3x = walkXPos4;
                    targetLeg3z = walkZPos4;
                    targetLeg4x = walkXPos8;
                    targetLeg4z = walkZPos8;
                    targetLeg1y = walkYPos8; 
                    if (currentLeg1x >= targetLeg1x) {
                        state = 0;
                        prevLeg1x = targetLeg1x;
                        prevLeg1z = targetLeg1z;
                        prevLeg2x = targetLeg2x;
                        prevLeg2z = targetLeg2z;
                        prevLeg3x = targetLeg3x;
                        prevLeg3z = targetLeg3z;
                        prevLeg4x = targetLeg4x;
                        prevLeg4z = targetLeg4z;
                        prevLeg1y = targetLeg1y;
                        // check we actually get there due to dividing errors
                        currentLeg1x = targetLeg1x;
                        currentLeg1z = targetLeg1z;
                        currentLeg2x = targetLeg2x;
                        currentLeg2z = targetLeg2z;
                        currentLeg3x = targetLeg3x;
                        currentLeg3z = targetLeg3z;
                        currentLeg4x = targetLeg4x;
                        currentLeg4z = targetLeg4z;
                        currentLeg1y = targetLeg1y;
                    }
                }
    
                
                stepDiffLeg1x = (targetLeg1x - prevLeg1x)/(5*rate);
                stepDiffLeg1z = (targetLeg1z - prevLeg1z)/(5*rate);
                currentLeg1x = currentLeg1x + stepDiffLeg1x;
                currentLeg1z = currentLeg1z + stepDiffLeg1z;
            
                stepDiffLeg2x = (targetLeg2x - prevLeg2x)/(5*rate);
                stepDiffLeg2z = (targetLeg2z - prevLeg2z)/(5*rate);
                currentLeg2x = currentLeg2x + stepDiffLeg2x;
                currentLeg2z = currentLeg2z + stepDiffLeg2z;
            
                stepDiffLeg3x = (targetLeg3x - prevLeg3x)/(5*rate);
                stepDiffLeg3z = (targetLeg3z - prevLeg3z)/(5*rate);
                currentLeg3x = currentLeg3x + stepDiffLeg3x;
                currentLeg3z = currentLeg3z + stepDiffLeg3z;
            
                stepDiffLeg4x = (targetLeg4x - prevLeg4x)/(5*rate);
                stepDiffLeg4z = (targetLeg4z - prevLeg4z)/(5*rate);
                currentLeg4x = currentLeg4x + stepDiffLeg4x;
                currentLeg4z = currentLeg4z + stepDiffLeg4z;
            
                stepDiffLeg1y = (targetLeg1y - prevLeg1y)/(5*rate);
                currentLeg1y = currentLeg1y + stepDiffLeg1y;
//            }   // end of state machine for walk test
    //        else { // Not walking, go back to default standing position
    //            initStart = 0;
    //            
    //            desired_roll = 0;
    //            desired_pitch = 0;
    //            desired_yaw = 0;
    //            
    //            targetLeg1x = 0;
    //            targetLeg1z = DEFAULT_Z;
    //            targetLeg2x = 0;
    //            targetLeg2z = DEFAULT_Z;
    //            targetLeg3x = 0;
    //            targetLeg3z = DEFAULT_Z;
    //            targetLeg4x = 0;
    //            targetLeg4z = DEFAULT_Z;
    //            targetLeg1y = 0;
    //        }
    
            
            // offsets to balance centre of gravity statically
            
            // x = map(RFB, -460,460,-20,20);   // front/back                  | Higher number moves the foot forward
            // x = constrain(x,-20,0);
            
             int offsetX = 0; //(offsetX - 25) - x;
             int offsetY = 35; // offsetY + 25;
            
            // Serial.print(pitch);
            // Serial.print(" , ");
            // Serial.print(roll);
            // Serial.print(" , ");
            // Serial.print(x);
            // Serial.print(" , ");
            // Serial.print(rate);
            // Serial.print(" , ");
            // Serial.print(targetLeg2x);
            // Serial.print(" , ");
            // Serial.print(currentLeg2x);
            // Serial.print(" , ");
            // Serial.print(targetLeg2z);
            // Serial.print(" , ");
            // Serial.print(currentLeg2z);
            
            // Serial.println(); 
            
    //        desired_pitch = 0; //-1.5;    // bodge to keep the nose up
            
            kinematic(LEG_FR, currentLeg1x+offsetX, currentLeg1y-offsetY, currentLeg1z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, currentLeg2x+offsetX, currentLeg1y+offsetY, currentLeg2z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, currentLeg3x+offsetX, currentLeg1y-offsetY, currentLeg3z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, currentLeg4x+offsetX, currentLeg1y+offsetY, currentLeg4z, desired_roll, desired_pitch, desired_yaw);
        }
        else if (test_mode == 's') {
            desired_x = 0;
            desired_y = 0;
            desired_z = DEFAULT_Z;
            desired_roll = 0;
            desired_pitch = 0;
            desired_yaw = 0; 
            
            kinematic(LEG_FR, 0, -DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            kinematic(LEG_FL, 0, DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            kinematic(LEG_BR, 0, -DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            kinematic(LEG_BL, 0, DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);

            incr = true;
        }
        else if (test_mode == ' ') {
            rest_position();
        }
        else if (test_mode == 'c') {
            reset_servos();   
        }
        else {
            kinematic(LEG_FR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_FL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BR, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            kinematic(LEG_BL, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);    
        }
    } // timed event
#endif
    
}
