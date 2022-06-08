#ifndef MINISPOT_H
#define MINISPOT_H

#define DEBUG 0

#define PWM_SERVO_ADDR 0x40

class Spot{
    private:
        //Nothing here
    public:
        //Constructor
        Spot(int x = 0, int y = 0, int z = 0);
        //Position
        int pos_x;
        int pos_y;
        int pos_z;
        //Angles
        int angle_x;
        int angle_y;
        int angle_z;
};

#endif 