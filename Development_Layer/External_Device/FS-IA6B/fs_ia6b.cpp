//
// Created by 24158 on 2026/6/17.
//

#include "fs_ia6b.h"
#include <stdio.h>

// [CN] 静态指针，用于回调函数中访问具体的类实例
static Class_FS_IA6B* Global_FS_IA6B_Instance = nullptr;

// [CN] 驱动层所需的回调函数桥接
static void FS_IA6B_Bridge_Callback(uint8_t* buf, uint16_t len) {
    if (Global_FS_IA6B_Instance) Global_FS_IA6B_Instance->UART_RxCpltCallback(buf,len);
}
//TODO：这部分内容还需要研究，貌似有：只支持一个实例、生命周期、Data_Process中就存在的竞态等问题
/**
 * @brief 遥控器FS_I6X/对应接收机初始化
 *
 * @param huart 指定的UART或者usart
 */
void Class_FS_IA6B::Init(UART_HandleTypeDef *huart)
{
    Global_FS_IA6B_Instance = this;
    UART_Manage_Object = UART_Init(huart, FS_IA6B_Bridge_Callback, FS_IA6B_FRAME_LENGTH);//Ibus一帧定长貌似为32

}

bool Class_FS_IA6B::Data_Process(uint8_t *Rx_Frame ){
//需要利用好返回值以用以解决存活判定
    uint16_t	calc_sum = 0;
    uint16_t 	ori_sum;
    uint8_t  	i;

    if( Rx_Frame[0] != 0x20 )
        return false;
    if( Rx_Frame[1] != 0x40 )
        return false;

    for( i = 0; i < 30; i++ )
        calc_sum += Rx_Frame[i];
    calc_sum ^= 0xFFFF;
    ori_sum = Rx_Frame[30] + (Rx_Frame[31]<<8);

    // printf( "calc_sum: %04x %04x\n", (unsigned)calc_sum, (unsigned)ori_sum );
    if( calc_sum != ori_sum )
        return false;

    // IBUS 协议：通道值 12 位 (0~4095)
    for (i = 0; i < 14; i++)
    {
        FS_IA6B_ibus_msg.ch[i] = Rx_Frame[i * 2 + 2] + ((Rx_Frame[i * 2 + 3] & 0x0F) << 8);
    }

    // 通道 15~18（FS-IA6B 无输出）
    for (i = 14; i < 18; i++)
    {
        FS_IA6B_ibus_msg.ch[i] = 0;
    }

    return true;
}

void Class_FS_IA6B::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length)
{
    if (Data_Process(Rx_Data) == true )//注意判定不要写反，这是导致时控时不能控的原因？--TODO：是否意味着丢包/错误解析率蛮高？貌似2s左右就至少有一次
    {
        Flag += 1;
        // 需要实现相关的前置判定，才能实现跳过该步骤以达成掉线保护，具体参考Class_VTM_RX中的实现？但貌似dr16中的未使用类似”接收不全“的判定？
        // 这正好是你 fs_ia6b.cpp:47 那句 想解决的问题 —— 答案是保留返回值，并且只在解析成功时才 Flag += 1。所以 Data_Process 不要改 void，保持 int（或改 bool 更语义化）。
    }


}

void Class_FS_IA6B::RTOS_100ms_Alive_Callback()
{
    static uint8_t cnt = 0;
    if (Flag == Pre_Flag)//HACK：貌似使用记录有效帧的方式会更加稳健：：Flag == Pre_Flag 这个判法本身很脆 —— 它依赖回调严格按 100ms 周期执行，且对"窗口边界"敏感（刚好在窗口切换时收到帧就可能误判）。用最后一次有效帧的时刻更直接：
    {
        FS_IA6B_Status = FS_IA6B_Status_DISABLE;
        //如果只依赖这个判定不能实现对发射端断控的检测，因为ia6b接收机在失去发射端发送数据时会一直发送上一次的有效帧
        //FS-i6X 的菜单里有 Failsafe 设置（失控保护），可以逐通道指定失联时的值。参考做法：
            //指定一个"使能开关"通道（比如 ch7 = SWD）
            //正常操作：SWD 打下 = 待机，推上 = 使能
            //把 failsafe 里 ch7 配成"打下"（待机位）
            //机器人侧：只有"帧有效 && ch7 == 使能"才驱动电机

        cnt++;
        if (cnt > 5)
        {
            cnt = 0;
            UART_Reinit(UART_Manage_Object->huart);
        }
    }
    else
    {
        cnt = 0;//解决自激震荡：cnt 在"成功"分支永不复位。所以即使修好上面的反条件，只要出现零星单窗口丢帧，cnt 就会累积，最终触发 UART_Reinit —— 而 reinit 会 __disable_irq + 停 DMA + 重挂，本身就会打乱接收、产生错位帧，于是又制造新的失败。
        if (FS_IA6B_ibus_msg.ch[5] >= 1751 )//HACK：待优化，对开关依赖性强（注意发射端对接收机设置的失控保护也需设置）
        {
            FS_IA6B_Status = FS_IA6B_Status_ENABLE;

        }else
        {
            FS_IA6B_Status = FS_IA6B_Status_DISABLE;
        }
    }

    Pre_Flag = Flag;
}

