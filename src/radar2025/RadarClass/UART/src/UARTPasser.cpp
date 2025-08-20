#include "../include/UARTPasser.h"

UARTPasser::UARTPasser()
{
}

UARTPasser::~UARTPasser()
{
}

int UARTPasser::bytes2Int(unsigned char a, unsigned char b)
{
    return (0x0000 | a) | (b << 8);
}

float UARTPasser::bytesToFloat(unsigned char bytes[])
{
    return *((float *)bytes);
}

void UARTPasser::push_loc(vector<vector<float>> &location)
{
    this->_robot_location.swap(location);
}

vector<vector<float>> UARTPasser::get_position()
{
    return this->_robot_location;
}

void UARTPasser::get_message()
{
    // TODO:信息获取接口
}

void UARTPasser::Refree_MapLocationSelf_Message()
{
    // TODO:位置获取接口
}

void UARTPasser::Referee_Update_GameData(unsigned char *buffer)
{
    if (this->_Now_stage < 2 && ((buffer[7] >> 4) == 2 || (buffer[7] >> 4) == 3 || (buffer[7] >> 4) == 4))
    {
        this->_Game_Start_Flag = true;
        this->_set_max_flag = true;
        this->Remain_time = 420;
        this->logger->critical("GAME START !");
    }
    if (this->_Now_stage < 5 && (buffer[7] >> 4) == 5)
    {
        this->_Game_End_Flag = true;
        this->logger->critical("GAME FINISH !");
        for (int i = 0; i < 12; ++i)
        {
            this->_max_hp[i] = this->_init_hp[i];
        }
        this->Remain_time = 0;
    }
    this->_Now_stage = buffer[7] >> 4;
}

void UARTPasser::Referee_Robot_HP(unsigned char *buffer)
{
    // 根据0x0003协议格式解析血量数据
    // 红1英雄机器人血量
    this->_HP[0] = this->bytes2Int(buffer[7], buffer[8]);
    
    // 红2工程机器人血量
    this->_HP[1] = this->bytes2Int(buffer[9], buffer[10]);
    
    // 红3步兵机器人血量
    this->_HP[2] = this->bytes2Int(buffer[11], buffer[12]);
    
    // 红4步兵机器人血量
    this->_HP[3] = this->bytes2Int(buffer[13], buffer[14]);
    
    // 保留位
    // this->_HP[4] = this->bytes2Int(buffer[15], buffer[16]);
    
    // 红7哨兵机器人血量
    this->_HP[5] = this->bytes2Int(buffer[17], buffer[18]);
    
    // 红方前哨站血量
    this->_HP[6] = this->bytes2Int(buffer[19], buffer[20]);
    
    // 红方基地血量
    this->_HP[7] = this->bytes2Int(buffer[21], buffer[22]);
    
    // 蓝1英雄机器人血量
    this->_HP[8] = this->bytes2Int(buffer[23], buffer[24]);
    
    // 蓝2工程机器人血量
    this->_HP[9] = this->bytes2Int(buffer[25], buffer[26]);
    
    // 蓝3步兵机器人血量
    this->_HP[10] = this->bytes2Int(buffer[27], buffer[28]);
    
    // 蓝4步兵机器人血量
    this->_HP[11] = this->bytes2Int(buffer[29], buffer[30]);
    
    // 保留位
    // this->_HP[12] = this->bytes2Int(buffer[31], buffer[32]);
    
    // 蓝7哨兵机器人血量
    this->_HP[13] = this->bytes2Int(buffer[33], buffer[34]);
    
    // 蓝方前哨站血量
    this->_HP[14] = this->bytes2Int(buffer[35], buffer[36]);
    
    // 蓝方基地血量
    this->_HP[15] = this->bytes2Int(buffer[37], buffer[38]);
}

BOData UARTPasser::One_compete_end()
{
    BOData temp;
    if (this->_Game_End_Flag)
    {
        this->_Game_End_Flag = false;
        ++this->_BO;
        temp.GameEndFlag = true;
        temp.remainBO = this->_BO - MAXBO;
    }
    return temp;
}

bool UARTPasser::One_compete_start()
{
    if (this->_Game_Start_Flag)
    {
        this->_Game_Start_Flag = false;
        return true;
    }
    else
        return false;
}

void UARTPasser::Receive_Robot_Data(unsigned char *buffer)
{
    if ((0x0000 | buffer[7]) | ((buffer[8] << 8) == 0x0200))
        this->logger->debug("Receive_Robot_Data");
}