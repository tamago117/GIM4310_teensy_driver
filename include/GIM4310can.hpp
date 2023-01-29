#pragma once
#include <FlexCAN.h>
#include "GIM4310can_code.hpp"

class GIM4310can
{
    public:
        GIM4310can();
        void turn_on();
        void turn_off();
        void set_zero();
        void position_control(float position);
        void velocity_control(float velocity);
        void current_control(float current);


    private:
        FlexCAN can;

};