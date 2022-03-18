#ifndef BK9_HARDWARE_H

// Pinout
#define FR_HIP_PIN      22
#define FR_SHOULDER_PIN 23
#define FR_KNEE_PIN     1

#define FL_HIP_PIN      0
#define FL_SHOULDER_PIN 8
#define FL_KNEE_PIN     7

#define BR_HIP_PIN      15
#define BR_SHOULDER_PIN 3 //14
#define BR_KNEE_PIN     13

#define BL_HIP_PIN      10
#define BL_SHOULDER_PIN 11
#define BL_KNEE_PIN     12

#define GIM_YAW_PIN     14 //3
#define GIM_PIT_PIN     2

#define V_DIV_PIN 21

// -----------------------------------------------------------
// Hardware replated parameters
#define MIN_SIG 500
#define MAX_SIG 2500
#define GIM_PIT_MIN 635
#define GIM_PIT_MAX 2460

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

#define VOLT_CONV_FACTOR 0.003107666016

// Gimble macros
#define GIM_YAW_STOP_SIG 95
#define GIM_YAW_CW_SIG 105
#define GIM_YAW_CCW_SIG 85
#define GIM_45_DEG_DELAY 350
// CW - sig > 97
// CCW - sig < 93

#define GIM_PIT_MIN 635
#define GIM_PIT_MAX 2460

#endif // BK9_HARDWARE_H
