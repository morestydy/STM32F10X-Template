# STM32

[toc]

## 1. 配置GPIO

**配置 步骤**

1. 开启 GPIO 时钟。

2. 第二步，选择需要配置的引脚。

3. 第三步，选择引脚速度（频率）。

4. 第四步，配置 GPIO 模式。

5. 第五步，初始化 GPIO。

   **led.c:**

```c
#include "led.h"
/*LED_G 驱动 GPIO 初始化函数*/
void led_gpio_config(void)
{
    GPIO_InitTypeDf GPIO_InitSructre;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_nitSructre.GPIO_Pin = GPIO_Pin_15;
    GPIO_nitSructre.GPIO_Sped = GPIO_Sped_50MHz;
    GPIO_nitSructre.GPIO_Mode = GPIO_Mode_Out_P;
    GPIO_nit(GPIOB, &GPIO_InitSructre);
}

```

**main.c:**

```c
#include "ld.h"
int main(void)
{
    led_gpio_config(); //配置 GPIO
    GPIO_SetBis(GPIOB, GPIO_Pin_15); //将 PB15 设置成高电平
    while(1); //等待
    return 0;
}
```

## 2. 配置SysTick

**配置步骤:**

1. 配置系统时钟；
2.  配置 SysTick；
3.  写 SysTick 中断处理函数；
4.  编写 delay 延迟函数；

**main.c:**

```c
#include "ld.h" #include "timer.h"
int main(void)
{
    SystemInit(); /初始化系统时钟
    Systick_Init(); /配置系统滴答
    led_gpio_config(); /配置 GPIO
    //SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    while(1)
    {
        GPIO_SetBis(GPIOB, GPIO_Pin_15); //将 PB15 设置成高电平
        delay_ms(10);
        GPIO_RestBis(GPIOB, GPIO_Pin_15); //将 PB15 设置成低电平
        delay_ms(10); //等待
    }
    return 0;
}

```

**timer.h**

```c
#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f10x_tim.h" #include "stm32f10x_rc.h" #include "stm32f10x_it.h" #include "misc.h" extrn __IO uint32_t TimngDelay;
void Systick_Init(void);
void TimngDelay_Decrment(void);
voidelay_ms(__IO uint32_t nTime); /延迟函数，设置为 ms
#endif

```

**timer.c**

```c
#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f10x_tim.h" #include "stm32f10x_rc.h" #include "stm32f10x_it.h" #include "misc.h" extrn __IO uint32_t TimngDelay;
void Systick_Init(void);
void TimngDelay_Decrment(void);
voidelay_ms(__IO uint32_t nTime); //延迟函数，设置为 ms
#endif
```

## 3. GPIO输入--按键查询

1. 配置GPIO设置为输入模式(第一节)

   **key.c**

   ```c
   #include "key.h"
   void key_gpio_init(void)
   {
     GPIO_InitTypeDef GPIO_InitSructure;
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
     GPIO_InitSructure.GPIO_Pin = GPIO_Pin_3;
     GPIO_InitSructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitSructure.GPIO_Mode = GPIO_Mode_IPU;
     GPIO_Init(GPIOE,&GPIO_InitSructure);
   } 
   ```

   **main.c:**

```c
int main(void)
{
    u8 led_state=0;
    SystemInit();                              //初始化系统，使得系统频率为72M
    systick_init();                            //配置Systick，使得1ms产生中断
    led_gpio_init();                          //配置LED驱动管脚
    key_gpio_init();                          //配置按键输入引脚
    while(1) 
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == RESET)
        {
            delay_ms (10);
            if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == RESET)
            {
              led_state = led_state >= 4 ? 0 : led_state + 1;
            }
            while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == RESET);
        }
        if((led_state % 2) == 1)
        {
            GPIO_SetBits (GPIOB, GPIO_Pin_5);   //点灯
        }
        else
        {
            GPIO_ResetBits (GPIOB, GPIO_Pin_5); //关灯
        }
    }
}
```

##  [GPIO的8种输入模式](https://blog.csdn.net/kevinhg/article/details/17490273)

一、推挽输出：

二、开漏输出：

三、浮空输入:

四、上拉输入/下拉输入/模拟输入：

五、复用开漏输出、复用推挽输出：

六、总结 在STM32中选用IO模式
    1、浮空输入GPIO_IN_FLOATING ——浮空输入，可以做KEY识别，RX1
    2、带上拉输入GPIO_IPU——IO内部上拉电阻输入
    3、带下拉输入GPIO_IPD—— IO内部下拉电阻输入
    4、模拟输入GPIO_AIN ——应用ADC模拟输入，或者低功耗下省电
    5、开漏输出GPIO_OUT_OD ——IO输出0接GND，IO输出1，悬空，需要外接上拉电阻，才能实现输出高电平。当输出为1时，IO口的状态由上拉电阻拉高电平，但由于是开漏输出模式，这样IO口也就可以由外部电路改变为低电平或不变。可以读IO输入电平变化，实现C51的IO双向功能
    6、推挽输出GPIO_OUT_PP ——IO输出0-接GND， IO输出1 -接VCC，读输入值是未知的
    7、复用功能的推挽输出GPIO_AF_PP ——片内外设功能（I2C的SCL,SDA）
    8、复用功能的开漏输出GPIO_AF_OD——片内外设功能（TX1,MOSI,MISO.SCK.SS）