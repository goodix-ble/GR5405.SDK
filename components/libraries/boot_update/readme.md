## 简介
本组件用于GR5xx芯片通过应用固件更新APP bootloader固件的场景。

## 使用方法
1.将boot_update.c\h加入APP工程编译

2.将新的boot固件转换常量并编译进APP固件

操作方法：新的boot固件(固件末尾添加固件信息)更改名字为：new_boot_fw.bin，并通过git bash将新的boot固件转换为new_boot_fw.h文件，命令如下

```
xxd -i new_boot_fw.bin | sed 's/unsigned char new_boot_fw_bin/\
#include "cmsis_compiler.h"\n__ALIGNED(256) const unsigned char new_boot_fw/' > new_boot_fw.h
```

3.调用boot_update_start函数

```
void boot_update_start(uint32_t old_boot_addr, uint32_t max_size, uint32_t new_boot_addr, uint32_t new_size);
```

示例代码：

```
#include "boot_update.h"
#include "new_boot_fw.h"
int main(void)
{
    // Initialize user peripherals.
    app_periph_init();
    // Feed watch dog if needed
    boot_update_start(0x00204000, 0x8000, (uint32_t)&new_boot_fw, sizeof(new_boot_fw));
    // loop
    while (1)
    {
        delay_ms(1000);
        printf("while loop \r\n");
        app_log_flush();
    }
}
```

说明：

1.调用boot_update_start前请先喂狗一次，并测量boot_update_start执行时间，确保看门狗时间足够，避免升级boot过程中被看门狗复位(GR5405芯片在系统时钟为64MHz, boot固件大小为32KB时，boot_update_start执行时间约150ms)

2.boot_update_start中会检查旧boot地址中的固件大小、名称以及校验和是否和APP中的boot相同，如果相同，则不会更新旧boot

3.更新boot成功后，默认会主动进行一次芯片复位

4.操作过程中不能断电，断电可能导致旧boot损坏，无法正常跳转APP固件

5.建议升级boot后，重新刷新为不包含升级boot的APP固件
