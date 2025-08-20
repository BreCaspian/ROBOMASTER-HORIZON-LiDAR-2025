#ifndef UARTTESTER_H
#define UARTTESTER_H

#include "../../Common/include/public.h"
#include "UART.h"
#include "serial.h"

/**
 * @brief 裁判系统测试类
 * 用于测试裁判系统的数据发送功能，完全符合新裁判系统0x0305协议
 */
class UARTTester
{
public:
    typedef std::shared_ptr<UARTTester> Ptr;

private:
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    UART::Ptr uart;
    MySerial::Ptr serial;
    bool isRunning = false;
    std::thread testThread;
    std::mutex testMutex;
    std::condition_variable testCV;
    int testInterval = 100; // 测试间隔，单位毫秒，默认10Hz
    
    // 测试数据
    struct TestVehicleData {
        int id;
        float x;
        float y;
    };
    
    std::vector<TestVehicleData> testVehicles = {
        {1, 12.5f, 8.3f},    // 红方英雄
        {2, 11.9f, 15.6f},   // 红方工程
        {3, 9.8f, 8.7f},     // 红方3号步兵
        {4, 8.2f, 3.7f},     // 红方4号步兵
        {5, 7.6f, 15.0f},    // 红方5号步兵
        {7, 10.0f, 9.3f},    // 红方哨兵
        {101, 18.5f, 18.3f}, // 蓝方英雄
        {102, 17.9f, 5.6f},  // 蓝方工程
        {103, 19.8f, 18.7f}, // 蓝方3号步兵
        {104, 18.2f, 13.7f}, // 蓝方4号步兵
        {105, 17.6f, 5.0f},  // 蓝方5号步兵
        {107, 20.0f, 9.3f}   // 蓝方哨兵
    };

    void testThreadFunction();
    void sendAllRobotPositions();

public:
    UARTTester(UART::Ptr uart, MySerial::Ptr serial);
    ~UARTTester();

    void start();
    void stop();
    bool isActive() const { return isRunning; }
    void setTestInterval(int intervalMs);
    int getTestInterval() const { return testInterval; }
};

#endif // UARTTESTER_H 