#ifndef C_BOARD_CLASS_TIMER_H
#define C_BOARD_CLASS_TIMER_H

#include "stm32h7xx_hal.h"

class User_Timer
{
public:
    User_Timer();

    void Get_Dt_Result();

    float dt;

private:
    static uint64_t Now_Ticks();

    uint64_t last_tick;
    bool     has_last_timepoint;
};

#endif // C_BOARD_CLASS_TIMER_H
