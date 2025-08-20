#include "../include/UARTTester.h"

UARTTester::UARTTester(UART::Ptr uart, MySerial::Ptr serial)
    : uart(uart), serial(serial)
{
    logger->info("UART Tester initialized");
}

UARTTester::~UARTTester()
{
    stop();
}

void UARTTester::start()
{
    std::unique_lock<std::mutex> lock(testMutex);
    if (!isRunning)
    {
        isRunning = true;
        testThread = std::thread(&UARTTester::testThreadFunction, this);
        logger->info("UART Test thread started");
    }
}

void UARTTester::stop()
{
    {
        std::unique_lock<std::mutex> lock(testMutex);
        if (isRunning)
        {
            isRunning = false;
            testCV.notify_all();
        }
    }
    
    if (testThread.joinable())
    {
        testThread.join();
        logger->info("UART Test thread stopped");
    }
}

void UARTTester::setTestInterval(int intervalMs)
{
    // 确保测试间隔不小于100毫秒(10Hz)，以符合协议要求
    if (intervalMs >= 100 && intervalMs <= 1000)
    {
        std::unique_lock<std::mutex> lock(testMutex);
        testInterval = intervalMs;
        logger->info("UART Test interval set to: {} ms", intervalMs);
    }
    else if (intervalMs < 100)
    {
        std::unique_lock<std::mutex> lock(testMutex);
        testInterval = 100;
        logger->info("UART Test interval set to minimum allowed: 100 ms (10Hz)");
    }
    else if (intervalMs > 1000)
    {
        std::unique_lock<std::mutex> lock(testMutex);
        testInterval = 1000;
        logger->info("UART Test interval set to maximum allowed: 1000 ms");
    }
}

void UARTTester::testThreadFunction()
{
    while (isRunning)
    {
        {
            std::unique_lock<std::mutex> lock(testMutex);
            if (!isRunning)
                break;
                
            // 发送所有机器人位置数据
            sendAllRobotPositions();
        }
        
        // 等待指定时间或者被通知停止
        std::unique_lock<std::mutex> lock(testMutex);
        // 确保测试间隔不小于100毫秒(10Hz)，以符合协议要求
        int actualInterval = std::max(100, testInterval);
        testCV.wait_for(lock, std::chrono::milliseconds(actualInterval), [this]() { return !isRunning; });
    }
}

// 按照新裁判系统协议发送所有机器人位置数据
void UARTTester::sendAllRobotPositions()
{
    if (uart)
    {
        // 创建位置数据
        std::vector<std::vector<float>> locations;
        
        // 初始化所有位置为0
        for (int i = 0; i < 6; ++i) {
            std::vector<float> pos = {0.0f, 0.0f};
            locations.push_back(pos);
        }
        
        // 根据敌方颜色选择要发送的车辆数据
        bool isRedEnemy = uart->ENEMY == 1;
        int startIndex = isRedEnemy ? 0 : 6; // 红方敌人发送红方数据，蓝方敌人发送蓝方数据
        
        // 设置英雄机器人位置
        locations[0][0] = testVehicles[startIndex].x;
        locations[0][1] = testVehicles[startIndex].y;
        
        // 设置工程机器人位置
        locations[1][0] = testVehicles[startIndex + 1].x;
        locations[1][1] = testVehicles[startIndex + 1].y;
        
        // 设置3号步兵位置
        locations[2][0] = testVehicles[startIndex + 2].x;
        locations[2][1] = testVehicles[startIndex + 2].y;
        
        // 设置4号步兵位置
        locations[3][0] = testVehicles[startIndex + 3].x;
        locations[3][1] = testVehicles[startIndex + 3].y;
        
        // 设置5号步兵位置
        locations[4][0] = testVehicles[startIndex + 4].x;
        locations[4][1] = testVehicles[startIndex + 4].y;
        
        // 设置哨兵位置
        locations[5][0] = testVehicles[startIndex + 5].x;
        locations[5][1] = testVehicles[startIndex + 5].y;
        
        // 通过UARTPasser更新位置数据
        uart->myUARTPasser.push_loc(locations);
        
        logger->info("Test: Updated UARTPasser with all robot positions for {} team", 
                   isRedEnemy ? "RED" : "BLUE");
    }
} 