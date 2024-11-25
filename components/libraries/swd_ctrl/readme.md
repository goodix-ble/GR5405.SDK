## 简介
为了支持SWD锁定情况下重新打开SWD并调试代码，本组件通过串口接收密钥方式重新打开SWD

## 使用方法
1.将swd_ctrl.c\h加入编译  
2.如果board_SK.h中未定义串口接收引脚或需要更改引脚，需要用户重定义SWD_CTRL_UART_CONFIG相关宏  
3.如果开启了看门狗，需要重新实现swd_unlock_wdt_refresh函数，避免等待接收pattern2时喂狗超时  
4. 调用swd_unlock_process函数  

### 配置密钥
在swd_ctrl.h中配置密钥PATTERN1和PATTERN2，默认为32字节，修改长度后，需要同步修改PATTERN_LENGTH。

### 配置超时时间
接收PATTERN1超时时间可配置，默认为1000ms，通过swd_ctrl.h中RX_PATTERN1_TIMEOUT修改

## SWD解锁操作步骤
1.连接PC串口和芯片串口以及SWD  
2.PC串口循环发送pattern1  
3.复位芯片  
4.从串口收到pattern1即立刻解锁SWD，并持续等待接收pattern2  
5.Jlink连接芯片（attach方式连接，不能复位芯片），如果不需要Jlink调试可跳过此步骤  
6.PC串口发送pattern2, 代码继续运行  

## 测试方法
硬件准备：串口线连接log串口RX引脚（SK板默认已连接）  
软件准备：调用sys_swd_disable()关闭SWD后调用swd_unlock_process()，测试代码 示例如下
```
#include "swd_ctrl.h"
int main (void)
{
    app_periph_init();
    delay_ms(3000); // 延时，避免测试异常时无法通过SWD下载代码，正式代码不需要
    sys_swd_disable(); // 关闭SWD, 模拟芯片SWD关闭场景，正式代码不需要
    swd_unlock_process();
    ...
}
```
