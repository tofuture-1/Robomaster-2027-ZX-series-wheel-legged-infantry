#include "referee.h"
#include "protocol_crc.h"
#include <string.h>
// [CN] 静态指针，用于回调函数中访问具体的类实例
static Class_Referee* Global_Referee_Instance = nullptr;

// [CN] 驱动层所需的回调函数桥接
static void Referee_Bridge_Callback(uint8_t* buf, uint16_t len) {
    if (Global_Referee_Instance) Global_Referee_Instance->UART_RxCpltCallback(buf, len);
}

// --- 类实现 ---

void Class_Referee::Init(UART_HandleTypeDef *huart) {
    Global_Referee_Instance = this;
    UART_Obj = UART_Init(huart, Referee_Bridge_Callback, 512);
}

void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length) {
    Flag++;
    Data_Process(Rx_Data, Length);
}

void Class_Referee::TIM_Alive_PeriodElapsedCallback() {
    Pre_Flag = Flag;
}

void Class_Referee::Data_Process(uint8_t *pData, uint16_t Len) {
    uint16_t i = 0;
    while (i + 9 <= Len) {
        if (pData[i] != 0xA5) { i++; continue; }
        // 校验头 CRC8
        if (!ProtocolCRC::VerifyCRC8CheckSum(&pData[i], 5, 0xFF)) { i++; continue; }

        uint16_t data_len = (pData[i + 2] << 8) | pData[i + 1];
        uint16_t packet_len = data_len + 9;
        if (i + packet_len > Len) break;

        // 校验整包 CRC16
        if (!ProtocolCRC::VerifyCRC16CheckSum(&pData[i], packet_len, 0xFFFF)) {
            i++; continue;
        }

        uint16_t cmd_id = (pData[i + 6] << 8) | pData[i + 5];
        uint8_t *data_ptr = &pData[i + 7];

        switch (cmd_id) {
            case Referee_Command_ID_GAME_STATUS:   memcpy(&Game_Status,  data_ptr, sizeof(Game_Status));   break;          // 0x0001 比赛状态 3Hz
            case Referee_Command_ID_GAME_RESULT:   memcpy(&Game_Result,  data_ptr, sizeof(Game_Result));   break;          // 0x0002 比赛结果 比赛结束
            case Referee_Command_ID_GAME_ROBOT_HP: memcpy(&Robot_HP,     data_ptr, sizeof(Robot_HP));     break;          // 0x0003 机器人血量 1Hz
            case Referee_Command_ID_EVENT_SELF_DATA:    memcpy(&Event_Data,   data_ptr, sizeof(Event_Data));    break;        // 0x0101 场地事件 1Hz
            case Referee_Command_ID_EVENT_SELF_REFEREE_WARNING: memcpy(&Referee_Warning, data_ptr, sizeof(Referee_Warning)); break; // 0x0104 裁判警告 判罚发生
            case Referee_Command_ID_EVENT_SELF_DART_STATUS: memcpy(&Dart_Status, data_ptr, sizeof(Dart_Status)); break;     // 0x0105 飞镖15s倒计时 1Hz
            case Referee_Command_ID_ROBOT_STATUS:  memcpy(&Robot_Status, data_ptr, sizeof(Robot_Status));  break;          // 0x0201 机器人状态 10Hz
            case Referee_Command_ID_ROBOT_POWER_HEAT:    memcpy(&Power_Heat,   data_ptr, sizeof(Power_Heat));    break;        // 0x0202 功率热量 50Hz
            case Referee_Command_ID_ROBOT_POSITION:     memcpy(&Robot_Pos,    data_ptr, sizeof(Robot_Pos));     break;         // 0x0203 机器人位置 10Hz
            case Referee_Command_ID_ROBOT_BUFF:    memcpy(&Robot_Buff,   data_ptr, sizeof(Robot_Buff));    break;          // 0x0204 机器人增益 1Hz
            case Referee_Command_ID_ROBOT_AERIAL_STATUS: memcpy(&Aerial_Status, data_ptr, sizeof(Aerial_Status)); break;     // 0x0205 空中机器人状态 10Hz
            case Referee_Command_ID_ROBOT_DAMAGE:     memcpy(&Hurt_Data,    data_ptr, sizeof(Hurt_Data));     break;          // 0x0206 伤害情况 伤害发生
            case Referee_Command_ID_ROBOT_BOOSTER:    memcpy(&Shoot_Data,   data_ptr, sizeof(Shoot_Data));    break;          // 0x0207 子弹信息 射击发生
            case Referee_Command_ID_ROBOT_REMAINING_AMMO:   memcpy(&Ammo_Remain,  data_ptr, sizeof(Ammo_Remain));   break;      // 0x0208 子弹剩余 10Hz
            case Referee_Command_ID_ROBOT_RFID:   memcpy(&RFID_Status,  data_ptr, sizeof(RFID_Status));   break;           // 0x0209 RFID状态 1Hz
            case Referee_Command_ID_ROBOT_DART_COMMAND: memcpy(&Dart_Command, data_ptr, sizeof(Dart_Command)); break;       // 0x020a 飞镖命令 10Hz
            case Referee_Command_ID_ROBOT_SENTRY_LOCATION: memcpy(&Sentry_Location, data_ptr, sizeof(Sentry_Location)); break; // 0x020b 哨兵位置 1Hz
            case Referee_Command_ID_ROBOT_RADAR_MARK: memcpy(&Radar_Mark, data_ptr, sizeof(Radar_Mark)); break;          // 0x020c 雷达标记 1Hz
            case Referee_Command_ID_ROBOT_SENTRY_DECISION: memcpy(&Sentry_Decision, data_ptr, sizeof(Sentry_Decision)); break; // 0x020d 哨兵决策 1Hz
            case Referee_Command_ID_ROBOT_RADAR_DECISION: memcpy(&Radar_Decision, data_ptr, sizeof(Radar_Decision)); break;  // 0x020e 雷达决策 1Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER: 
                memcpy(&Custom_Controller_Data, data_ptr, sizeof(Custom_Controller_Data)); break; // 0x0302 自定义控制器 30Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP:
                memcpy(&Client_Minimap, data_ptr, sizeof(Client_Minimap)); break; // 0x0303 客户端小地图 2Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL:
                memcpy(&Client_Remote_Control, data_ptr, sizeof(Client_Remote_Control)); break; // 0x0304 图传键鼠遥控 30Hz
            default: break;
        }
        i += packet_len;
    }
}

void Class_Referee::Send_UI_Graphic_1(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 15;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 15);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_String(uint8_t layer, const char name[3], const char* str, uint16_t start_x, uint16_t start_y) {
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 15 + 30;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);

    Struct_Referee_Data_Interaction_Graphic_Config config{};
    memcpy(config.Index, name, 3);
    config.Figure_Type = 7; config.Layer = layer; config.Start_X = start_x; config.Start_Y = start_y;
    config.Details_A = 20; config.Details_B = strlen(str); config.Details_C = 2; config.Operation = 1;

    memcpy(&Tx_Buffer[13], &config, 15);
    strncpy((char*)&Tx_Buffer[28], str, 30);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_2(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制2个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0101
    // 数据段: 6字节帧头 + 30字节图形数据(2个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 30;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 30);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_5(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制5个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0102
    // 数据段: 6字节帧头 + 75字节图形数据(5个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 75;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 75);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_7(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制7个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0103
    // 数据段: 6字节帧头 + 105字节图形数据(7个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 105;
    uint16_t total_len = data_len + 9;

    if (total_len > sizeof(Tx_Buffer)) return;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 105);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Layer_Delete(uint8_t layer, uint8_t operation, uint8_t delete_serial) {
    // [CN] 删除指定图层的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0100
    // 数据段: 6字节帧头 + 3字节操作数据(operation + layer + delete_serial)
    // operation: 1-删除单个, 2-删除全部
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 3;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_LAYER_DELETE;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    Tx_Buffer[13] = operation;
    Tx_Buffer[14] = layer;
    Tx_Buffer[15] = delete_serial;

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Sentry_Decision(uint8_t confirm_respawn, uint8_t confirm_exchange_respawn, 
                                         uint16_t ammo_number, uint8_t remote_exchange_ammo_times, 
                                         uint8_t remote_exchange_hp_times, uint8_t sentry_mode,
                                         uint8_t confirm_activate_energy) {
    // [CN] 哨兵自主决策交互信息发送
    // cmd_id: 0x0301, sub_cmd_id: 0x0120
    // 数据段：6 字节帧头 + 4 字节决策数据
    // 决策数据位域: 
    //   bit 0: 确认复活
    //   bit 1: 确认兑换立即复活
    //   bit 2-12: 弹药数量 (0-2047)
    //   bit 13-16: 远程兑换弹药量请求次数
    //   bit 17-20: 远程兑换血量请求次数
    //   bit 21-22: 哨兵姿态 (1-进攻，2-防御，3-移动)
    //   bit 23: 确认激活能量机关
    //   bit 24-31: 保留位
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 4;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_SENTRY;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_Server;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);

    uint32_t decision_data = 0;
    decision_data |= (confirm_respawn & 0x01);                                    // bit 0
    decision_data |= ((confirm_exchange_respawn & 0x01) << 1);                    // bit 1
    decision_data |= ((ammo_number & 0x7FF) << 2);                                // bit 2-12
    decision_data |= ((remote_exchange_ammo_times & 0x0F) << 13);                 // bit 13-16
    decision_data |= ((remote_exchange_hp_times & 0x0F) << 17);                   // bit 17-20
    decision_data |= ((sentry_mode & 0x03) << 21);                                // bit 21-22
    decision_data |= ((confirm_activate_energy & 0x01) << 23);                    // bit 23
    // bit 24-31: Reserved (already 0)
    memcpy(&Tx_Buffer[13], &decision_data, 4);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Radar_Decision(uint8_t request_double_damage) {
    // [CN] 雷达自主决策交互信息发送
    // cmd_id: 0x0301, sub_cmd_id: 0x0121
    // 数据段: 6字节帧头 + 1字节请求数据
    // 请求数据: bit0-1双倍伤害几率(0-3), bit2双倍伤害敌方状态
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 1;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_RADAR;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_Server;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    Tx_Buffer[13] = request_double_damage;

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Custom_Controller_Data(uint8_t *data, uint8_t len) {
    // [CN] 自定义控制器交互数据发送
    // cmd_id: 0x0302
    // 数据段: 6字节帧头 + 自定义数据(最大30字节)
    // 发送频率: 最高30Hz
    if (UART_Obj == nullptr || len > 30) return;
    uint16_t data_len = 6 + len;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    memcpy(&Tx_Buffer[11], data, len);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Client_Receive_Radar(uint16_t *robot_positions, uint8_t len) {
    // [CN] 雷达发送机器人位置信息给客户端
    // cmd_id: 0x0305
    // 数据段：6 字节帧头 + 24 字节位置数据
    // 位置数据：每个机器人 4 字节 (x 2 字节 + y 2 字节)，单位 cm
    // 发送频率：最高 10Hz
    // 参数：robot_positions - uint16_t 数组，包含 12 个位置坐标 (6 个机器人的 x,y)
    //       len - 数据长度 (固定为 24 字节)
    if (UART_Obj == nullptr || len != 24) return;
    uint16_t data_len = 6 + 24;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_All;  // 发送给所有客户端
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    
    // 复制 24 字节位置数据
    memcpy(&Tx_Buffer[11], robot_positions, 24);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Robot_Minimap(uint16_t sender_id, uint16_t receiver_id, uint8_t *data, uint8_t len) {
    // [CN] 机器人发送小地图数据给客户端
    // cmd_id: 0x0308
    // 数据段: 6字节帧头 + 4字节标识 + 自定义数据(最大30字节)
    // 发送频率: 最高10Hz
    if (UART_Obj == nullptr || len > 30) return;
    uint16_t data_len = 6 + 4 + len;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    memcpy(&Tx_Buffer[11], data, len);

    ProtocolCRC::AppendCRC8CRC16CheckSum(Tx_Buffer, total_len);
    UART_Send_Data(UART_Obj->huart, Tx_Buffer, total_len);
}


