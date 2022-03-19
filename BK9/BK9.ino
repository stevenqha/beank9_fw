#include <Servo.h>
#include <Ramp.h>
#include "BK9_Hardware.h"
#include "Inverse_Kinematics.h"

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


leg_t leg_fr = {.leg = LEG_FR};
leg_t leg_fl = {.leg = LEG_FL};
leg_t leg_br = {.leg = LEG_BR};
leg_t leg_bl = {.leg = LEG_BL};

Servo gim_yaw_servo;
Servo gim_pitch_servo;




#define BUF_LENGTH 10
#define DEFAULT_Z 200 // mm (results in 90 degrees between thigh and shin)
#define DEFAULT_Y_OFFSET 10 // mm





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


void reset_servos() {
    leg_fr.hip.writeMicroseconds(1500);
    leg_fr.shoulder.writeMicroseconds(2500);
    leg_fr.knee.writeMicroseconds(1500);

    leg_fl.hip.writeMicroseconds(1500);
    leg_fl.shoulder.writeMicroseconds(500);
    leg_fl.knee.writeMicroseconds(1500);

    leg_br.hip.writeMicroseconds(1500);
    leg_br.shoulder.writeMicroseconds(500);
    leg_br.knee.writeMicroseconds(1500);

    leg_bl.hip.writeMicroseconds(1500);
    leg_bl.shoulder.writeMicroseconds(2500);
    leg_bl.knee.writeMicroseconds(1500);
}

void rest_position() {
//    inv_kin(&leg_fr, -43.5, -DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    inv_kin(&leg_fl, -43.5, DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    inv_kin(&leg_br, 43.5, -DEFAULT_Y_OFFSET, 115, 0, 0, 0);
//    inv_kin(&leg_bl, 43.5, DEFAULT_Y_OFFSET, 115, 0, 0, 0);

    inv_kin(&leg_fr, -43.5, 0, 115, 0, 0, 0);
    inv_kin(&leg_fl, -43.5, 0, 115, 0, 0, 0);
    inv_kin(&leg_br, 43.5, 0, 115, 0, 0, 0);
    inv_kin(&leg_bl, 43.5, 0, 115, 0, 0, 0);
    
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
        
        inv_kin(&leg_fr, -x_val, DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        inv_kin(&leg_fl, -x_val, -DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        inv_kin(&leg_br, x_val, DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        inv_kin(&leg_bl, x_val, -DEFAULT_Y_OFFSET, z_val, 0, 0, 0);
        
        delay(10);
    }
}

int gim_pitch_deg;

void gim_rotate(bool clockwise, bool up) {
    rampInt pitch_pos;
    int pitch_sig = 0;
    int target;
    
//    if(clockwise) {
//        gim_yaw_servo.write(GIM_YAW_CW_SIG);
//        delay(10);
//    }
//    else {
//        gim_yaw_servo.write(GIM_YAW_CCW_SIG);
//        delay(10);
//    }

    if(up) {
//        pitch_pos.go(180);
//        pitch_pos.go(135, 290, LINEAR, ONCEFORWARD);

        target = 45;
        pitch_pos.go(0);
        pitch_pos.go(45, 290, LINEAR, ONCEFORWARD);
    }
    else {
//        pitch_pos.go(135);
//        pitch_pos.go(180, 290, LINEAR, ONCEFORWARD);

        target = 0;
        pitch_pos.go(45);
        pitch_pos.go(0, 290, LINEAR, ONCEFORWARD);
    }

    if (gim_pitch_deg != target) {
        for (int i = 0; i < 34; i++) {
    
            gim_pitch_servo.write(pitch_pos.update());
            delay(10);
        }
    }

    gim_pitch_deg = target;

//    gim_yaw_servo.write(GIM_YAW_STOP_SIG);
}

#define RESET 0
#define DEMO 1
#define BATTERY_MEASURE 1

int desired_z = DEFAULT_Z; // 90 degree between joints
int desired_x = 0;
int desired_y = 0;
float desired_pitch = 0;
float desired_roll = 0;
float desired_yaw = 0;

unsigned long currentMillis;
unsigned long previousMillis = 0;
unsigned long prev_batt_millis = 0;
bool toggle_walk = false;
bool incr = true;


rampInt test_ramp;
int test_mode = -1;


void setup() {

    leg_fr.hip.attach(FR_HIP_PIN, MIN_SIG, MAX_SIG);
    leg_fr.shoulder.attach(FR_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    leg_fr.knee.attach(FR_KNEE_PIN, MIN_SIG, MAX_SIG);

    leg_fl.hip.attach(FL_HIP_PIN, MIN_SIG, MAX_SIG);
    leg_fl.shoulder.attach(FL_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    leg_fl.knee.attach(FL_KNEE_PIN, MIN_SIG, MAX_SIG);

    leg_br.hip.attach(BR_HIP_PIN, MIN_SIG, MAX_SIG);
    leg_br.shoulder.attach(BR_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    leg_br.knee.attach(BR_KNEE_PIN, MIN_SIG, MAX_SIG);

    leg_bl.hip.attach(BL_HIP_PIN, MIN_SIG, MAX_SIG);
    leg_bl.shoulder.attach(BL_SHOULDER_PIN, MIN_SIG, MAX_SIG);
    leg_bl.knee.attach(BL_KNEE_PIN, MIN_SIG, MAX_SIG);

    gim_pitch_servo.attach(GIM_PIT_PIN, GIM_PIT_MIN, GIM_PIT_MAX);
    gim_pitch_servo.write(0);
    gim_pitch_deg = 0;

//    gim_yaw_servo.attach(GIM_YAW_PIN);
//    gim_yaw_servo.write(GIM_YAW_STOP_SIG);

    analogReadResolution(12);
    
    Serial.begin(115200);
    while(!Serial) {}

    #if RESET
    //reset_servos();
    #else
    rest_position();

//    // Wait for command from Jetson to stand
//    while(!Serial.available());
//
//    while(Serial.available() > 0) {
//        char t = Serial.read();
//    }
    
    stand();

    #endif // RESET
    
    delay(1000);
}

void loop() {
    // Check if new commands
    numBytes = Serial.available();
    if(readline(Serial.read(), buffer, BUF_LENGTH) > 0) {
        #if !DEMO
//        desired_z = atoi(buffer);
//        Serial.println(desired_z);
//        desired_x = atoi(buffer);
//        Serial.println(desired_x);
        desired_y = atoi(buffer);
        Serial.println(desired_y);
//        desired_roll = atoi(buffer);
//        Serial.println(desired_roll);
//        desired_pitch = atoi(buffer);
//        Serial.println(desired_pitch);
//        desired_yaw = atoi(buffer);
//        Serial.println(desired_yaw);

//        gimble_pitch = atoi(buffer);
//        Serial.println(gimble_pitch);

        inv_kin(&leg_fr, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        inv_kin(&leg_fl, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        inv_kin(&leg_br, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
        inv_kin(&leg_bl, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);

//        gim_pitch_servo.writeMicroseconds(gimble_pitch);

        #else
        if (buffer[0] == 'x' || buffer[0] == 'y' || buffer[0] == 'z' || 
            buffer[0] == 'r' || buffer[0] == 'p' || buffer[0] == 'h' ||
            buffer[0] == 'w' || buffer[0] == 's' || buffer[0] == ' ' ||
            buffer[0] == 'c' || buffer[0] == 'u' || buffer[0] == 'g' || 't') {

            if (buffer[0] == 'g') {
                int cmd = atoi(buffer + 1);
                Serial.println(cmd);
                
                if (cmd == MOVE_CW_UP) {
                    Serial.println("move_cw_up");
                    gim_rotate(ROT_CW, MOV_UP);
                }
                else if(cmd == MOVE_CCW_UP) {
                    Serial.println("move_cww_up");
                    gim_rotate(ROT_CCW, MOV_UP);
                }
                else if(cmd == MOVE_CW_DOWN) {
                    Serial.println("move_cw_down");
                    gim_rotate(ROT_CW, MOV_DOWN);
                }
                else if(cmd == MOVE_CCW_DOWN) {
                    Serial.println("move_ccw_down");
                    gim_rotate(ROT_CCW, MOV_DOWN);
                } 
            }
            else { // Update test mode and echo it
                test_mode = buffer[0];
    
                Serial.println(buffer[0]);
            }    
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
        #endif // DEMO
    }

    currentMillis = millis();

#if DEMO
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

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

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

            inv_kin(&leg_fr, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, desired_y, desired_z, desired_roll, desired_pitch, desired_yaw);

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

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

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

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

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

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

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

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

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
            
                else if (state == 1) {  // &leg_bl Lifting
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
            
            inv_kin(&leg_fr, currentLeg1x+offsetX, currentLeg1y+offsetY, currentLeg1z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, currentLeg2x+offsetX, currentLeg1y-offsetY, currentLeg2z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, currentLeg3x+offsetX, currentLeg1y+offsetY, currentLeg3z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, currentLeg4x+offsetX, currentLeg1y-offsetY, currentLeg4z, desired_roll, desired_pitch, desired_yaw);
        }
        else if (test_mode == 's') {
            desired_x = 0;
            desired_y = 0;
            desired_z = DEFAULT_Z;
            desired_roll = 0;
            desired_pitch = 0;
            desired_yaw = 0; 
            
            inv_kin(&leg_fr, 0, DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            inv_kin(&leg_fl, 0, -DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            inv_kin(&leg_br, 0, DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);
            inv_kin(&leg_bl, 0, -DEFAULT_Y_OFFSET, DEFAULT_Z, 0, 0, 0);

            incr = true;
        }
        else if (test_mode == ' ') {
            rest_position();
        }
        else if (test_mode == 'c') {
            reset_servos();
        }
        else if (test_mode == 'u') {
            stand();
            test_mode = 's';
        }
        else if (test_mode == 't') {
            desired_z = 200;
            
            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, 115, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(1000);

            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, 200, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);

            delay(1000);

            test_mode = 's';
        }
        else {
            inv_kin(&leg_fr, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_fl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_br, desired_x, DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);
            inv_kin(&leg_bl, desired_x, -DEFAULT_Y_OFFSET, desired_z, desired_roll, desired_pitch, desired_yaw);    
        }
    } // timed event
#endif

#if BATTERY_MEASURE
    if (currentMillis - prev_batt_millis >= BATTERY_MEASURE_PERIOD_MS) {
        prev_batt_millis = currentMillis;
        int val = analogRead(V_DIV_PIN);
        double batt_voltage = (val) * VOLT_CONV_FACTOR;
        Serial.println(batt_voltage, 2);
    }
#endif // BATTERY_MEASURE
}
