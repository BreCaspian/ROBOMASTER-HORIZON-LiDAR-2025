#ifndef SERIAL_H
#define SERIAL_H

#include "../../Common/include/public.h"

/**
 * @brief 串口类
 * 用于串口通讯
 */
class MySerial
{
public:
    typedef std::shared_ptr<MySerial> Ptr;

    int fd = -1;
    int flag;
    int wr_num = 0;
    int rr_num = 0;
    struct termios options, newstate;
    speed_t baud_rate_i, baud_rate_o;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    std::mutex writeMutex; // 添加互斥锁保护串口写入操作

public:
    MySerial();
    ~MySerial();

    void initSerial(std::string sername, std::string password);
    bool _is_open();

    void msread(unsigned char buffer[], size_t size);
    void mswrite(unsigned char buffer[], size_t size);
};

#endif