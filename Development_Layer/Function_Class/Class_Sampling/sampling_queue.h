//
// Created by 谭恩泽 on 2025/10/23.
//

#ifndef C_BOARD_SAMPLING_QUEUE_H
#define C_BOARD_SAMPLING_QUEUE_H


#include "stdint.h"

/**
 * @brief Reusable, 循环队列本体
 *
 */
template<typename Type, uint16_t Max_Size = 200>
class Class_Queue
{
public:
    void Init();

    inline uint16_t Get_Length();

    inline Type Get_Front();

    inline Type Get_Rear();

    inline void Push(Type __Data);

    inline Type Pop();

    inline void Clear();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 队列本体
    Type Queue[Max_Size];

    // 队首元素位置
    uint32_t Front = 0;
    // 队尾元素位置
    uint32_t Rear = 0;

    // 读变量

    // 队列长度
    uint32_t Length = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化
 *
 * @tparam Type 类型
 * @tparam Max_Size 队列长度最大值
 */
template<typename Type, uint16_t Max_Size>
void Class_Queue<Type, Max_Size>::Init()
{
    Front = 0;
    Rear = 0;
    Length = 0;
}

/**
* @brief 获取队列长度
 *
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 队首元素
*/
template<typename Type, uint16_t Max_Size>
inline uint16_t Class_Queue<Type, Max_Size>::Get_Length()
{
    return (Length);
}

/**
* @brief 获取队首元素
 *
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 队首元素
*/
template<typename Type, uint16_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Get_Front()
{
    return (Queue[Front]);
}

/**
 * @brief 获取队尾元素
 *
 * @tparam Type 类型
 * @tparam Max_Size 队列长度最大值
 * @return 队尾元素
 */
template<typename Type, uint16_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Get_Rear()
{
    return (Queue[Rear]);
}

/**
* @brief 送入队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @param __Data 传入值
*/
template<typename Type, uint16_t Max_Size>
inline void Class_Queue<Type, Max_Size>::Push(Type __Data)
{
    if (Length == Max_Size)
    {
        return;
    }
    else
    {
        if (Length == 0)
        {
            Front = 0;
            Rear = 0;
        }
        else
        {
            Rear = (Rear + 1) % Max_Size;
        }

        Queue[Rear] = __Data;
        Length++;
    }
}

/**
* @brief 弹出队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 弹出值
*/
template<typename Type, uint16_t Max_Size>
inline Type Class_Queue<Type, Max_Size>::Pop()
{
    Type temp = Queue[Front];

    if (Length == 0)
    {

    }
    else
    {

        Front = (Front + 1) % Max_Size;
        Length--;
    }

    return (temp);
}

/**
* @brief 清空队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
*/
template<typename Type, uint16_t Max_Size>
inline void Class_Queue<Type, Max_Size>::Clear()
{
    Front = 0;
    Rear = 0;
    Length = 0;
}



#endif //C_BOARD_SAMPLING_QUEUE_H
