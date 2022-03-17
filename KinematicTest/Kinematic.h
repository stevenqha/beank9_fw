//
// Created by happy on 2022-03-15.
//

#ifndef KINEMATICTEST_KINEMATIC_H
#define KINEMATICTEST_KINEMATIC_H

typedef enum {
    LEG_FR,
    LEG_FL,
    LEG_BR,
    LEG_BL
} leg_e;

void kinematic(leg_e leg, float x, float y, float z, float roll, float pitch, float yaw);

#endif //KINEMATICTEST_KINEMATIC_H
