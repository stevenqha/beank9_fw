#include <iostream>
#include "Kinematic.h"

int main() {
//    // Testing pitch
//    kinematic(LEG_FR, 0, 0, 200, 0, 31, 0);
//    kinematic(LEG_BR, 0, 0, 200, 0, 31, 0);
//
//    kinematic(LEG_FR, 10, 0, 200, 0, 0, 0);
//    kinematic(LEG_BR, 10, 0, 200, 0, 0, 0);

    // Testing roll
    kinematic(LEG_FR, 0, 0, 200, 31, 0, 0);
    kinematic(LEG_FL, 0, 0, 200, 31, 0, 0);

    kinematic(LEG_FR, 0, 20, 200, 31, 0, 0);
    kinematic(LEG_FL, 0, 20, 200, 31, 0, 0);

    return 0;
}
