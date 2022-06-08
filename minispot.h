#ifndef MINISPOT_H
#define MINISPOT_H

#define PWM_SERVO_ADDR 0x40

namespace SPOT{

typdef struct spot{

    //Position
    int pos_x;
    int pos_y;
    int pos_z;
    //Angles
    int angle_x;
    int angle_y;
    int angle_z;

}spot

}

#endif 