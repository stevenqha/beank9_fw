#include <Servo.h>

typedef enum {
    LEG_FR,
    LEG_FL,
    LEG_BR,
    LEG_BL
} leg_e;

typedef struct leg_t {
    leg_e leg;
    Servo hip;
    Servo shoulder;
    Servo knee;
} leg_t;

// Gimble enums
typedef enum {
    MOVE_CW_UP,
    MOVE_CCW_UP,
    MOVE_CW_DOWN,
    MOVE_CCW_DOWN    
} gim_e;

typedef enum {
    ROT_CCW,
    ROT_CW
} rot_e;

typedef enum {
    MOV_DOWN,
    MOV_UP
} mov_e;

#define BATTERY_MEASURE_PERIOD_MS 10000
