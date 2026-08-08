#include "class_timer.h"

User_Timer::User_Timer()
{
    last_tick          = 0U;
    has_last_timepoint = false;
    dt                 = 0.0f;
}

// 用 SysTick 的 uwTick + VAL 合成单调递增的 64 位 tick 计数器
// SysTick 每 tick = 1 / SystemCoreClock 秒（CLKSOURCE = HCLK，不分频）
uint64_t User_Timer::Now_Ticks()
{
    uint32_t ms, val;
    do {
        ms  = uwTick;
        val = SysTick->VAL;
    } while (ms != uwTick);           // 确保 ms 和 val 属于同一个 tick 周期

    uint64_t r = (uint64_t)SysTick->LOAD + 1ULL;          // 每 ms 的 tick 数
    uint64_t e = r - 1ULL - (uint64_t)val;               // 当前 ms 内已走过的 tick 数
    return (uint64_t)ms * r + e;
}

void User_Timer::Get_Dt_Result()
{
    uint64_t now = Now_Ticks();

    if (!has_last_timepoint) {
        last_tick          = now;
        has_last_timepoint = true;
        dt = 0.0f;
        return;
    }

    // delta_tick / SystemCoreClock → 秒，精度 1 个 HCLK 周期 (~7ns)
    dt = (float)(now - last_tick) / (float)SystemCoreClock;
    last_tick = now;
}
