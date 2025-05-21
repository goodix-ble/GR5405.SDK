## 简介
本组件实现了通过BLE GATT服务进行芯片调试功能

具有如下功能：

- 读写SRAM
- 读写寄存器
- 读取FLASH

## 使用方法
将ble_debug.c\h, ble_debug_server.c\h加入工程编译，调用ble_debug_service_init()注册服务，并开启广播和配对功能，通过特定的BLE调试工具与芯片连接配对完成后，即可进行调试.
示例工程：SDK\projects\ble\example\ble_debug_demo


