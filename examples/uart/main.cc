/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "main.h"

#include <memory>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

/**
 * 用于验证此示例程序传输正确性的Python客户端示例代码
 *
 * 功能说明：此Python代码通过串口发送数据，并验证是否能正确接收到3倍长度的回传数据
 *
 * ```
 * import serial
 *
 * ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
 * some_str = 'this is my data!'
 *
 * for i in range(1000):
 *     ser.write(some_str)  # 发送测试字符串
 *     while ser.in_waiting < 3 * len(some_str):  # 等待接收3倍长度的数据
 *         continue
 *     ret = ser.read_all()  # 读取所有接收到的数据
 *     assert ret == 3 * some_str  # 验证接收到的数据是否为发送数据的3倍
 * ```
 */

// 定义接收信号标志，用于线程间通信
#define RX_SIGNAL (1 << 0)

// 外部声明的默认任务句柄
extern osThreadId_t defaultTaskHandle;

/**
 * 自定义UART类，继承自基础的bsp::UART类
 * 主要功能：重写接收完成回调函数，实现数据接收通知机制
 */
class CustomUART : public bsp::UART {
public:
    // 使用基类的构造函数
    using bsp::UART::UART;

protected:
    /**
     * 接收完成回调函数
     * 当有待读取的接收数据时通知应用程序
     * 通过设置线程标志来唤醒主任务进行数据处理
     */
    void RxCompleteCallback() override final {
        osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL);
    }
};

/**
 * RTOS默认任务函数
 * 主要功能：实现串口数据的接收和三次回传
 *
 * @param argument 任务参数（未使用）
 */
extern "C" void RTOS_Default_Task(const void* argument) {
    UNUSED(argument);  // 标记参数未使用，避免编译器警告

    uint32_t length;   // 接收到的数据长度
    uint8_t* data;     // 指向接收数据的指针

    // 创建自定义UART对象的智能指针，UART_HANDLE在cmake中定义具体使用哪个串口
    auto uart = std::make_unique<CustomUART>(&UART_HANDLE);
    uart->SetupRx(50);  // 设置接收缓冲区大小为50字节
    uart->SetupTx(50);  // 设置发送缓冲区大小为50字节

    // 主循环：持续监听串口数据
    while (true) {
        // 等待接收数据信号，使用阻塞等待直到有数据到达
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);

        if (flags & RX_SIGNAL) {  // 检查是否收到接收信号（这个检查在技术上是多余的）
            // 执行非阻塞的接收和发送操作（执行时间应该 <= 1个OS时钟周期）
            length = uart->Read(&data);     // 读取接收到的数据
            uart->Write(data, length);      // 第一次回传相同的数据
            uart->Write(data, length);      // 第二次回传相同的数据
            uart->Write(data, length);      // 第三次回传相同的数据

            // 总共发送3倍的接收数据，实现数据回传功能
        }
    }
}