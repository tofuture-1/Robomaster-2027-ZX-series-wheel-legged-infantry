//
// Created by 谭恩泽 on 2025/10/30.
//

#ifndef C_BOARD_RTOS_TASK_H
#define C_BOARD_RTOS_TASK_H


#ifdef __cplusplus

#include "callback_task.h"

#endif


// 区分C和C++编译器：C++编译器会定义__cplusplus宏
#ifdef __cplusplus
extern "C" {  // 告诉C++编译器，内部代码按C规则编译
#endif

    //初始化
    void Robot_Init_Task();
	
    void Robot_Main_Loop();


#ifdef __cplusplus
}  // 结束extern "C"
#endif



#endif //C_BOARD_RTOS_TASK_H
