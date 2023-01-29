#pragma once
#include <Arduino.h>

enum class operation_mode{
    position,
    velocity,
    current
};

namespace GIM4310code_param{
    const float P_MAX = 32768;
    const float V_MAX = 2048;
    const float C_MAX = 2048;
    const float KP_MAX = 4096;
    const float KD_MAX = 4096;
}

namespace GIM4310_param{
    const float P_MAX = 12.5;
    const float V_MAX = 65;
    const float C_MAX = 4;
    const float KP_MAX = 500;
    const float KD_MAX = 5;
}

uint16_t float_to_uint(float v, float v_min, float v_max, uint32_t width)
{
    float temp;
    int32_t utemp;

    //temp = ((v-v_min)/(v_max-v_min))*((float)width);
    //utemp = (int32_t)temp;
    utemp = (int16_t)v;

    if(utemp < 0){
        utemp = 0;
    }
    if(utemp > width){
        utemp = width;
    }

    return utemp;
}

char* turn_on_motor(bool mode)
{
    char transmit_message[8];

    if(mode){
        transmit_message[0] = 0xFF;
        transmit_message[1] = 0xFF;
        transmit_message[2] = 0xFF;
        transmit_message[3] = 0xFF;
        transmit_message[4] = 0xFF;
        transmit_message[5] = 0xFF;
        transmit_message[6] = 0xFF;
        transmit_message[7] = 0xFC;
    }else{
        transmit_message[0] = 0xFF;
        transmit_message[1] = 0xFF;
        transmit_message[2] = 0xFF;
        transmit_message[3] = 0xFF;
        transmit_message[4] = 0xFF;
        transmit_message[5] = 0xFF;
        transmit_message[6] = 0xFF;
        transmit_message[7] = 0xFD;
    }

    return transmit_message;
}

char* set_zero_position(void)
{
    char transmit_message[8];

    transmit_message[0] = 0xFF;
    transmit_message[1] = 0xFF;
    transmit_message[2] = 0xFF;
    transmit_message[3] = 0xFF;
    transmit_message[4] = 0xFF;
    transmit_message[5] = 0xFF;
    transmit_message[6] = 0xFF;
    transmit_message[7] = 0xFE;

    return transmit_message;
}

char* switch_operation_mode(operation_mode mode)
{
    char transmit_message[8];

    switch(mode)
    {
        case operation_mode::position:
            transmit_message[0] = 0xFF;
            transmit_message[1] = 0xFF;
            transmit_message[2] = 0xFF;
            transmit_message[3] = 0xFF;
            transmit_message[4] = 0xFF;
            transmit_message[5] = 0xFF;
            transmit_message[6] = 0xFF;
            transmit_message[7] = 0xFB;
            break;
        case operation_mode::velocity:
            transmit_message[0] = 0xFF;
            transmit_message[1] = 0xFF;
            transmit_message[2] = 0xFF;
            transmit_message[3] = 0xFF;
            transmit_message[4] = 0xFF;
            transmit_message[5] = 0xFF;
            transmit_message[6] = 0xFF;
            transmit_message[7] = 0xFA;
            break;
        case operation_mode::current:
            transmit_message[0] = 0xFF;
            transmit_message[1] = 0xFF;
            transmit_message[2] = 0xFF;
            transmit_message[3] = 0xFF;
            transmit_message[4] = 0xFF;
            transmit_message[5] = 0xFF;
            transmit_message[6] = 0xFF;
            transmit_message[7] = 0xF9;
            break;
        default:
            break;
    }

    return transmit_message;
}

// rad -12.5 ~ 12.5 kp 0 ~ 500 kd 0 ~ 5
char* set_position(float position, float kp, float kd)
{
    char transmit_message[8];

    float a1 = GIM4310_param::P_MAX;
    float b1 = GIM4310code_param::P_MAX;
    float pos_code = position*b1/a1 + b1;

    float a2 = GIM4310_param::KP_MAX;
    float b2 = GIM4310code_param::KP_MAX;
    float kp_code = kp*b2/a2 + b2;

    float a3 = GIM4310_param::KD_MAX;
    float b3 = GIM4310code_param::KD_MAX;
    float kd_code = kd*b3/a3 + b3;

    uint16_t s_p_int = float_to_uint(pos_code, -GIM4310code_param::P_MAX, GIM4310code_param::P_MAX, GIM4310code_param::P_MAX*2);
    uint16_t s_v_int = float_to_uint(0, -GIM4310code_param::V_MAX, GIM4310code_param::V_MAX, GIM4310code_param::V_MAX*2);
    uint16_t s_Kp_int = float_to_uint(kp_code, 0 , GIM4310code_param::KP_MAX, GIM4310code_param::KP_MAX*2);
    uint16_t s_Kd_int = float_to_uint(kd_code, 0 , GIM4310code_param::KD_MAX, GIM4310code_param::KD_MAX*2);
    uint16_t s_c_int = float_to_uint(0, -GIM4310code_param::C_MAX, GIM4310code_param::C_MAX, GIM4310code_param::C_MAX*2);

    transmit_message[0] = s_p_int>>8;
    transmit_message[1] = s_p_int&0xFF;
    transmit_message[2] = s_v_int>>4;
    transmit_message[3] = ((s_v_int&0xF)<<4) + (s_Kp_int >>8);
    transmit_message[4] = s_Kp_int &0xFF;
    transmit_message[5] = s_Kd_int>>4;
    transmit_message[6] = ((s_Kd_int &0xF)<<4) + (s_c_int >>8);
    transmit_message[7] = s_c_int&0xFF;

    return transmit_message;
}

// rad/s -65 ~ 65
char* set_velocity(float velocity)
{
    char transmit_message[8];

    float a = GIM4310_param::V_MAX;
    float b = GIM4310code_param::V_MAX;
    float vel_code = velocity*b/a + b;

    uint16_t s_p_int = float_to_uint(0, -GIM4310code_param::P_MAX, GIM4310code_param::P_MAX, GIM4310code_param::P_MAX*2);
    uint16_t s_v_int = float_to_uint(vel_code, -GIM4310code_param::V_MAX, GIM4310code_param::V_MAX, GIM4310code_param::V_MAX*2);
    uint16_t s_Kp_int = float_to_uint(0, 0 , GIM4310code_param::KP_MAX, GIM4310code_param::KP_MAX*2);
    uint16_t s_Kd_int = float_to_uint(0, 0 , GIM4310code_param::KD_MAX, GIM4310code_param::KD_MAX*2);
    uint16_t s_c_int = float_to_uint(0, -GIM4310code_param::C_MAX, GIM4310code_param::C_MAX, GIM4310code_param::C_MAX*2);

    transmit_message[0] = s_p_int>>8;
    transmit_message[1] = s_p_int&0xFF;
    transmit_message[2] = s_v_int>>4;
    transmit_message[3] = ((s_v_int&0xF)<<4) + (s_Kp_int >>8);
    transmit_message[4] = s_Kp_int &0xFF;
    transmit_message[5] = s_Kd_int>>4;
    transmit_message[6] = ((s_Kd_int &0xF)<<4) + (s_c_int >>8);
    transmit_message[7] = s_c_int&0xFF;

    return transmit_message;
}

// amp -4 ~ 4
char* set_current(float current)
{
    char transmit_message[8];

    float a = GIM4310_param::C_MAX;
    float b = GIM4310code_param::C_MAX;
    float current_code = current*b/a + b;

    uint16_t s_p_int = float_to_uint(0, -GIM4310code_param::P_MAX, GIM4310code_param::P_MAX, GIM4310code_param::P_MAX*2);
    uint16_t s_v_int = float_to_uint(0, -GIM4310code_param::V_MAX, GIM4310code_param::V_MAX, GIM4310code_param::V_MAX*2);
    uint16_t s_Kp_int = float_to_uint(0, 0 , GIM4310code_param::KP_MAX, GIM4310code_param::KP_MAX*2);
    uint16_t s_Kd_int = float_to_uint(0, 0 , GIM4310code_param::KD_MAX, GIM4310code_param::KD_MAX*2);
    uint16_t s_c_int = float_to_uint(current_code, -GIM4310code_param::C_MAX, GIM4310code_param::C_MAX, GIM4310code_param::C_MAX*2);

    transmit_message[0] = s_p_int>>8;
    transmit_message[1] = s_p_int&0xFF;
    transmit_message[2] = s_v_int>>4;
    transmit_message[3] = ((s_v_int&0xF)<<4) + (s_Kp_int >>8);
    transmit_message[4] = s_Kp_int &0xFF;
    transmit_message[5] = s_Kd_int>>4;
    transmit_message[6] = ((s_Kd_int &0xF)<<4) + (s_c_int >>8);
    transmit_message[7] = s_c_int&0xFF;

    return transmit_message;
}

float get_position(const char message[7])
{
    /*
    float a = GIM4310_param::C_MAX;
    float b = GIM4310code_param::C_MAX;
    float current_code = current*b/a + b;

    uint16_t s_p_int = float_to_uint(0, -GIM4310code_param::P_MAX, GIM4310code_param::P_MAX, GIM4310code_param::P_MAX*2);
    uint16_t s_v_int = float_to_uint(0, -GIM4310code_param::V_MAX, GIM4310code_param::V_MAX, GIM4310code_param::V_MAX*2);
    uint16_t s_Kp_int = float_to_uint(0, 0 , GIM4310code_param::KP_MAX, GIM4310code_param::KP_MAX*2);
    uint16_t s_Kd_int = float_to_uint(0, 0 , GIM4310code_param::KD_MAX, GIM4310code_param::KD_MAX*2);
    uint16_t s_c_int = float_to_uint(current_code, -GIM4310code_param::C_MAX, GIM4310code_param::C_MAX, GIM4310code_param::C_MAX*2);

    transmit_message[0] = s_p_int>>8;
    transmit_message[1] = s_p_int&0xFF;
    transmit_message[2] = s_v_int>>4;
    transmit_message[3] = ((s_v_int&0xF)<<4) + (s_Kp_int >>8);
    transmit_message[4] = s_Kp_int &0xFF;
    transmit_message[5] = s_Kd_int>>4;
    transmit_message[6] = ((s_Kd_int &0xF)<<4) + (s_c_int >>8);
    transmit_message[7] = s_c_int&0xFF;
    */

    int motor_id = message[0] & 0xFF;
    float pos_code = (((((message[1])) << 8) & 0x0000FF00) | ((((message[2])) << 0) & 0x000000FF));
    float vel_code = (((((message[3])) << 4) & 0x00000FF0) | ((((message[4])) >> 4) & 0x0000000F));
    float current_code = (((((message[4])) << 8) & 0x00000F00) | ((((message[5])) << 0) & 0x000000FF));

    float position = (pos_code-GIM4310code_param::P_MAX)/GIM4310code_param::P_MAX*GIM4310_param::P_MAX;
    float velocity = (vel_code-GIM4310code_param::V_MAX)/GIM4310code_param::V_MAX*GIM4310_param::V_MAX;
    float current = (current_code-GIM4310code_param::C_MAX)/GIM4310code_param::C_MAX*GIM4310_param::C_MAX;

    return position;
}