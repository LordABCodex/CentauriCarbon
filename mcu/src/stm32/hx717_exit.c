#include "hx717_exit.h"
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "generic/armcm_timer.h" // udelay
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include <string.h> // ffs
#include "sched.h" // sched_shutdown
#include "generic/armcm_boot.h" // armcm_enable_irq
#define FTIR 0x01
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
//ex_NVIC_config专用定义
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6 
#define GPIO_H 				7 
#define GPIO_I 				8 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#define HX717_NUM_1                 0
#define HX717_NUM_2                 1
#define HX717_NUM_3                 2
#define HX717_NUM_4                 3



static struct gpio_out clks[4];
// struct gpio_in sdos[4];

#define GPIO_PORT_DOUT1             GPIOB
#define GPIO_PIN_DOUT1              (1 << 14)
#define EXIT_PIN_DOUT1              14
#define GPIO_PORT_PDSCK1            GPIOB
#define GPIO_PIN_PDSCK1             (1 << 13)
#define KLIPPER_GPIO_PIN_PDSCK1     29




#define GPIO_PORT_DOUT2             GPIOC
#define GPIO_PIN_DOUT2              (1 << 8)
#define EXIT_PIN_DOUT2              8
#define GPIO_PORT_PDSCK2            GPIOC
#define GPIO_PIN_PDSCK2             (1 << 7)
#define KLIPPER_GPIO_PIN_PDSCK2     39

#define GPIO_PORT_DOUT3             GPIOB
#define GPIO_PIN_DOUT3              (1 << 15)
#define EXIT_PIN_DOUT3              15
#define GPIO_PORT_PDSCK3            GPIOC
#define GPIO_PIN_PDSCK3             (1 << 6)
#define KLIPPER_GPIO_PIN_PDSCK3     38

#define GPIO_PORT_DOUT4             GPIOA
#define GPIO_PIN_DOUT4              (1 << 8)
#define EXIT_PIN_DOUT4              8
#define GPIO_PORT_PDSCK4            GPIOC
#define GPIO_PIN_PDSCK4             (1 << 9)
#define KLIPPER_GPIO_PIN_PDSCK4     41

#define GPIO_NUM_DOUT1              1   // GPIOB
#define GPIO_NUM_DOUT2              2   // GPIOC
#define GPIO_NUM_DOUT3              1   // GPIOB
#define GPIO_NUM_DOUT4              0   // GPIOA

// IO编号和EXTIx_Handler编号绑定，修改IO时注意修改EXTI_Init里面的EXTIx_Handler号
#define PDSCK1     PBout(13)
#define DOUT1      PBin(14)
#define PDSCK2     PCout(7)
#define DOUT2      PCin(8)
#define PDSCK3     PCout(6)
#define DOUT3      PBin(15)
#define PDSCK4     PCout(9)
#define DOUT4      PAin(8)

#define LED      PAout(5)

volatile int32_t g_hx711_exi_data[5] = {0}; 
volatile uint64_t g_hx711_exi_tick[5] = {0}; 

void ex_NVIC_config(uint8_t GPIOx, uint8_t BITx, uint8_t TRIM)
{
    uint8_t EXTOFFSET = (BITx % 4) * 4;
    RCC->APB2ENR |= 1 << 14;        // 使能SYSCFG时钟
    SYSCFG->EXTICR[BITx / 4] &= ~(0x000F << EXTOFFSET); // 清除原来设置
    SYSCFG ->EXTICR[BITx / 4] |= GPIOx << EXTOFFSET; // EXTI.BITx映射到GPIOx.BITx
    // 自动设置
    EXTI->IMR |= 1 << BITx;     // 开启line BITx上的中断（如果要禁止中断,则反向操作即可）
    if(TRIM & 0x01)
    {
        EXTI->FTSR |= 1 << BITx;        // 下降沿触发
    }
    if(TRIM & 0x02)
    {
        EXTI->RTSR |= 1 << BITx;        // 上升沿触发
    }
}
//设置向量表偏移地址
//NVIC_VectTab:基址
//Offset:偏移量		 
void my_NVIC_set_vectortable(uint32_t NVIC_VectTab,uint32_t Offset)	 
{ 	   	  
	SCB->VTOR=NVIC_VectTab|(Offset&(uint32_t)0xFFFFFE00);//设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留。
}
//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组 		   
void my_NVIC_priority_group_config(uint8_t NVIC_Group)	 
{ 
	uint32_t temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先	   
void my_NVIC_init(uint8_t NVIC_PreemptionPriority,uint8_t NVIC_SubPriority,uint8_t NVIC_Channel,uint8_t NVIC_Group)	 
{ 
	uint32_t temp;	  
	my_NVIC_priority_group_config(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;								//取低四位
	NVIC->ISER[NVIC_Channel/32]|=1<<NVIC_Channel%32;//使能中断位(要清除的话,设置ICER对应位为1即可)
	NVIC->IP[NVIC_Channel]|=temp<<4;				//设置响应优先级和抢断优先级   	    	  				   
} 
//GPIO通用设置 
//GPIOx:GPIOA~GPIOI.
//BITx:0X0000~0XFFFF,位设置,每个位代表一个IO,第0位代表Px0,第1位代表Px1,依次类推.比如0X0101,代表同时设置Px0和Px8.
//MODE:0~3;模式选择,0,输入(系统复位默认状态);1,普通输出;2,复用功能;3,模拟输入.
//OTYPE:0/1;输出类型选择,0,推挽输出;1,开漏输出.
//OSPEED:0~3;输出速度设置,0,2Mhz;1,25Mhz;2,50Mhz;3,100Mh. 
//PUPD:0~3:上下拉设置,0,不带上下拉;1,上拉;2,下拉;3,保留.
//注意:在输入模式(普通输入/模拟输入)下,OTYPE和OSPEED参数无效!!
void gpio_set(GPIO_TypeDef* GPIOx,uint32_t BITx,uint32_t MODE,uint32_t OTYPE,uint32_t OSPEED,uint32_t PUPD)
{  
	uint32_t pinpos=0,pos=0,curpin=0;
	for(pinpos=0;pinpos<16;pinpos++)
	{
		pos=1<<pinpos;	//一个个位检查 
		curpin=BITx&pos;//检查引脚是否要设置
		if(curpin==pos)	//需要设置
		{
			GPIOx->MODER&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->MODER|=MODE<<(pinpos*2);	//设置新的模式 
			if((MODE==0X01)||(MODE==0X02))	//如果是输出模式/复用功能模式
			{  
				GPIOx->OSPEEDR&=~(3<<(pinpos*2));	//清除原来的设置
				GPIOx->OSPEEDR|=(OSPEED<<(pinpos*2));//设置新的速度值  
				GPIOx->OTYPER&=~(1<<pinpos) ;		//清除原来的设置
				GPIOx->OTYPER|=OTYPE<<pinpos;		//设置新的输出模式
			}  
			GPIOx->PUPDR&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->PUPDR|=PUPD<<(pinpos*2);	//设置新的上下拉
		}
	}
} 
void DOUT_input_mode(uint8_t hx717_num)
{
    uint32_t Pin_Num = 2;
    uint8_t GPIO_MODE_IN = 0;  // 输入IO
    uint8_t GPIO_PUPD_PU = 1;   // 1 上拉，2 下来    
    switch(hx717_num)
    {
        case HX717_NUM_1:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT1;     //使能PORT时钟 
            gpio_set(GPIO_PORT_DOUT1,GPIO_PIN_DOUT1,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PE2~4设置上拉输入
            break;
        case HX717_NUM_2:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT2;     //使能PORT时钟 
            gpio_set(GPIO_PORT_DOUT2,GPIO_PIN_DOUT2,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PE2~4设置上拉输入
            break;     
        case HX717_NUM_3:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT3;     //使能PORT时钟 
            gpio_set(GPIO_PORT_DOUT3,GPIO_PIN_DOUT3,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PE2~4设置上拉输入
            break;      
        case HX717_NUM_4:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT4;     //使能PORT时钟 
            gpio_set(GPIO_PORT_DOUT4,GPIO_PIN_DOUT4,GPIO_MODE_IN,0,0,GPIO_PUPD_PU);	//PE2~4设置上拉输入
            break;                                 
        default:
            break;
    }
    
    
}
void my_delay_us(uint32_t us)
{
    uint32_t delay = us * 80 / 4;
    do
    {
        __NOP();
    }while(delay --);
}

// 56789
void EXTI9_5_IRQHandler(void)
{
    int32_t count = 0; 
    static uint32_t loop2 = 0;
    static int32_t last_count2 = 0;
    static uint32_t err_count2 = 0;
    static uint32_t loop4 = 0;
    static int32_t last_count4 = 0;
    static uint32_t err_count4 = 0; 
    uint8_t i;
    static uint64_t last_tick =0;
    uint64_t tick = 0;
    uint32_t now_tick = timer_read_time();
    uint64_t clock_diff = (last_tick - (uint64_t)now_tick) & 0xffffffff;
    if (clock_diff & 0x80000000)
    {
        tick = last_tick + 0x100000000 - clock_diff;
    } else {
        tick = last_tick - clock_diff;
    }
    last_tick = tick;
    // UART1_Printf("EXTI9_5_IRQHandler\r\n");
    if((EXTI->PR >> EXIT_PIN_DOUT2) & 0x01)
    {
        loop2++;
        DOUT_input_mode(HX717_NUM_2);
        count = 0; 
        for(i = 0; i < 24; i++)
        {
            PDSCK2 = 1;
            delay_nus(1);
            count = count << 1; 
            PDSCK2 = 0; 
            if(1 == DOUT2)
            {
                count ++; 
            }
            delay_nus(1);
        }
        PDSCK2 = 1;
        delay_nus(1);
        count |= ((count & 0x00800000) != 0 ? 0xFF000000 : 0); 
        PDSCK2 = 0;
        EXTI_init(HX717_NUM_2);
        EXTI->PR = 1 << EXIT_PIN_DOUT2;      // 清除LINE1上的中断标志位
        g_hx711_exi_data[HX717_NUM_2] = count;
        g_hx711_exi_tick[HX717_NUM_2] = tick;
        if (count-last_count2> 10000 || count-last_count2 < -10000) {
            err_count2 ++;
            if (err_count2%100 == 0)
                UART1_Printf("hx717-2 %d %d/%d\r\n", count,err_count2,loop2);
        }
        last_count2 = count;
    }
    if((EXTI->PR >> EXIT_PIN_DOUT4) & 0x01)
    {
        loop4++;
        DOUT_input_mode(HX717_NUM_4);
        count = 0;
        for(i = 0; i < 24; i++)
        {
            PDSCK4 = 1;
            delay_nus(1);
            count = count << 1;
            PDSCK4 = 0; 
            if(1 == DOUT4)
            {
                count ++;
            }
            delay_nus(1);
        }
        PDSCK4 = 1;
        delay_nus(1);
        count |= ((count & 0x00800000) != 0 ? 0xFF000000 : 0);
        PDSCK4 = 0;
        EXTI_init(HX717_NUM_4);
        EXTI->PR = 1 << EXIT_PIN_DOUT4;      // 清除LINE1上的中断标志位
        g_hx711_exi_data[HX717_NUM_4] = count;
        g_hx711_exi_tick[HX717_NUM_4] = tick;
        if (count-last_count4> 10000 || count-last_count4 < -10000) {
            err_count4 ++;
            if (err_count4%100 == 0)
                UART1_Printf("hx717-4 %d %d/%d\r\n", count,err_count4,loop4);
        }
        last_count4 = count;
    } 
}
//10~15
void EXTI15_10_IRQHandler(void)
{
    int32_t count = 0; 
    uint8_t i;
    static int32_t last_count1 = 0;
    static int32_t last_count3 = 0;
    static uint32_t err_count1 = 0;
    static uint32_t err_count3 = 0; 
    static uint32_t loop1 = 0;
    static uint32_t loop3 = 0;
    static uint64_t last_tick =0;
    uint64_t tick = 0;
    uint32_t now_tick = timer_read_time();
    uint64_t clock_diff = (last_tick - (uint64_t)now_tick) & 0xffffffff;
    if (clock_diff & 0x80000000)
    {
        tick = last_tick + 0x100000000 - clock_diff;
    } else {
        tick = last_tick - clock_diff;
    }
    last_tick = tick;

    if((EXTI->PR >> EXIT_PIN_DOUT1) & 0x01)
    {
        loop1++;
        DOUT_input_mode(HX717_NUM_1);
        count = 0;
        for(i = 0; i < 24; i++)
        {
            PDSCK1 = 1;
            delay_nus(1);
            count = count << 1;
            PDSCK1 = 0; 
            if(1 == DOUT1)
            {
                count ++;
            }
            delay_nus(1);
        }
        PDSCK1 = 1;
        delay_nus(1);
        count |= ((count & 0x00800000) != 0 ? 0xFF000000 : 0);
        PDSCK1 = 0;
        EXTI_init(HX717_NUM_1);
        EXTI->PR = 1 << EXIT_PIN_DOUT1;      // 清除LINE1上的中断标志位
        g_hx711_exi_data[HX717_NUM_1] = count;
        g_hx711_exi_tick[HX717_NUM_1] = tick;
        if (count-last_count1> 10000 || count-last_count1 < -10000) {
            err_count1 ++;
            if (err_count1%100 == 0)
                UART1_Printf("hx717-1 %d %d/%d\r\n", count,err_count1,loop1);
        }
        last_count1 = count;
    }
    if((EXTI->PR >> EXIT_PIN_DOUT3) & 0x01)
    {
        loop3++;
        DOUT_input_mode(HX717_NUM_3);
        count = 0;
        for(i = 0; i < 24; i++)
        {
            PDSCK3 = 1;
            delay_nus(1);
            count = count << 1;
            PDSCK3 = 0;
            if(1 == DOUT3)
            {
                count ++;
            }
            delay_nus(1);
        }
        PDSCK3 = 1;
        delay_nus(1);
        count |= ((count & 0x00800000) != 0 ? 0xFF000000 : 0);
        PDSCK3 = 0;
        EXTI_init(HX717_NUM_3);
        EXTI->PR = 1 << EXIT_PIN_DOUT3;      // 清除LINE1上的中断标志位
        g_hx711_exi_data[HX717_NUM_3] = count;
        g_hx711_exi_tick[HX717_NUM_3] = tick;
        if (count-last_count3> 10000 || count-last_count3 < -10000) {
            err_count3 ++;
            if (err_count3%100 == 0)
                UART1_Printf("hx717-3 %d %d/%d\r\n", count,err_count3,loop3);
        }
        last_count3 = count;
    }  
}
void EXTI_init(uint8_t hx717_num)
{
    switch(hx717_num)
    {
        case HX717_NUM_1:  
            ex_NVIC_config(GPIO_NUM_DOUT1,EXIT_PIN_DOUT1,FTIR); 		//下降沿触发
            // my_NVIC_init(3,1,EXTI15_10_IRQHandler,2);		//抢占3，子优先级1，组2
            armcm_enable_irq(EXTI15_10_IRQHandler, EXTI15_10_IRQn, 0);
            break;
        case HX717_NUM_2:  
            ex_NVIC_config(GPIO_NUM_DOUT2,EXIT_PIN_DOUT2,FTIR); 		//下降沿触发
            // my_NVIC_init(3,2,EXTI9_5_IRQHandler,2);		//抢占3，子优先级2，组2  
            armcm_enable_irq(EXTI9_5_IRQHandler, EXTI9_5_IRQn, 0);
            break;         
        case HX717_NUM_3:  
            ex_NVIC_config(GPIO_NUM_DOUT3,EXIT_PIN_DOUT3,FTIR); 		//下降沿触发
            // my_NVIC_init(3,1,EXTI15_10_IRQHandler,2);		//抢占3，子优先级1，组2 
            armcm_enable_irq(EXTI15_10_IRQHandler, EXTI15_10_IRQn, 0);
            break;
        case HX717_NUM_4:  
            ex_NVIC_config(GPIO_NUM_DOUT4,EXIT_PIN_DOUT4,FTIR); 		//下降沿触发
            // my_NVIC_init(3,2,EXTI9_5_IRQHandler,2);		//抢占3，子优先级2，组2 
            armcm_enable_irq(EXTI9_5_IRQHandler, EXTI9_5_IRQn, 0);
            break;             
        default:
            break;
    }

}

void SCK_gpio_init(uint8_t hx717_num)
{
    switch(hx717_num)
    {
        case HX717_NUM_1:
            clks[HX717_NUM_1] = gpio_out_setup(KLIPPER_GPIO_PIN_PDSCK1,0);
            UART1_Printf("PDSCK1 = 0\r\n");   
            break; 
        case HX717_NUM_2:   
            clks[HX717_NUM_2] = gpio_out_setup(KLIPPER_GPIO_PIN_PDSCK2,0);
            UART1_Printf("PDSCK2 = 0\r\n");   
            break;
        case HX717_NUM_3: 
            clks[HX717_NUM_3] = gpio_out_setup(KLIPPER_GPIO_PIN_PDSCK3,0);
            UART1_Printf("PDSCK3 = 0\r\n");     
            break;
        case HX717_NUM_4:    
            clks[HX717_NUM_4] = gpio_out_setup(KLIPPER_GPIO_PIN_PDSCK4,0);
            UART1_Printf("PDSCK4 = 0\r\n");   
            break;
        default:
            break;
    }
	
}

void SCK_init(uint8_t hx717_num)
{
    uint32_t Pin_Num = 3;
    uint8_t GPIO_MODE_OUT = 1;  // 输出IO
    uint8_t GPIO_OTYPE_PP = 0;  //推挽输出
    uint8_t GPIO_SPEED_50M = 2;
    uint8_t GPIO_PUPD_PU = 1;   // 1 上拉，2 下来
    switch(hx717_num)
    {
        case HX717_NUM_1:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT1;//使能PORTC时钟 
            gpio_set(GPIO_PORT_PDSCK1,GPIO_PIN_PDSCK1,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU); //设置
            PDSCK1 = 0;
            UART1_Printf("PDSCK1 = 0\r\n");   
            break; 
        case HX717_NUM_2:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT2;//使能PORTC时钟 
            gpio_set(GPIO_PORT_PDSCK2,GPIO_PIN_PDSCK2,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU); //设置
            PDSCK2 = 0;   
            UART1_Printf("PDSCK2 = 0\r\n");   
            break;
        case HX717_NUM_3:
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT3;//使能PORTC时钟 
            gpio_set(GPIO_PORT_PDSCK3,GPIO_PIN_PDSCK3,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU); //设置   
            PDSCK3 = 0;   
            UART1_Printf("PDSCK3 = 0\r\n");     
            break;
        case HX717_NUM_4:   
            RCC->AHB1ENR|=1<<GPIO_NUM_DOUT4;//使能PORTC时钟 
            gpio_set(GPIO_PORT_PDSCK4,GPIO_PIN_PDSCK4,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU); //设置   
            PDSCK4 = 0;     
            UART1_Printf("PDSCK4 = 0\r\n");   
            break;
        default:
            break;
    }
	
}
//设置向量表偏移地址
//NVIC_VectTab:基址
//Offset:偏移量		 
void MY_NVIC_SetVectorTable(uint32_t NVIC_VectTab,uint32_t Offset)	 
{ 	   	  
	SCB->VTOR=NVIC_VectTab|(Offset&(uint32_t)0xFFFFFE00);//设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留。
}
//系统软复位   
void Sys_Soft_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	  
} 		 
//时钟设置函数
//Fvco=Fs*(plln/pllm);
//Fsys=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//Fsys:系统时钟频率
//Fusb:USB,SDIO,RNG等的时钟频率
//Fs:PLL输入时钟频率,可以是HSI,HSE等. 
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.

//外部晶振为8M的时候,推荐值:plln=336,pllm=8,pllp=2,pllq=7.
//得到:Fvco=8*(336/8)=336Mhz
//     Fsys=336/2=168Mhz
//     Fusb=336/7=48Mhz
//返回值:0,成功;1,失败。
uint8_t Sys_Clock_Set(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{ 
	uint16_t retry=0;
	uint8_t status=0;
	RCC->CR|=1<<16;				//HSE 开启 
	while(((RCC->CR&(1<<17))==0)&&(retry<0X1FFF))retry++;//等待HSE RDY
	if(retry==0X1FFF)status=1;	//HSE无法就绪
	else   
	{
		RCC->APB1ENR|=1<<28;	//电源接口时钟使能
		PWR->CR|=3<<14; 		//高性能模式,时钟可到168Mhz
		RCC->CFGR|=(0<<4)|(5<<10)|(4<<13);//HCLK 不分频;APB1 4分频;APB2 2分频. 
		RCC->CR&=~(1<<24);	//关闭主PLL
		RCC->PLLCFGR=pllm|(plln<<6)|(((pllp>>1)-1)<<16)|(pllq<<24)|(1<<22);//配置主PLL,PLL时钟源来自HSE
		RCC->CR|=1<<24;			//打开主PLL
		while((RCC->CR&(1<<25))==0);//等待PLL准备好 
		FLASH->ACR|=1<<8;		//指令预取使能.
		FLASH->ACR|=1<<9;		//指令cache使能.
		FLASH->ACR|=1<<10;		//数据cache使能.
		FLASH->ACR|=5<<0;		//5个CPU等待周期. 
		RCC->CFGR&=~(3<<0);		//清零
		RCC->CFGR|=2<<0;		//选择主PLL作为系统时钟	 
		while((RCC->CFGR&(3<<2))!=(2<<2));//等待主PLL作为系统时钟成功. 
	} 
	return status;
}  
//系统时钟初始化函数
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.
//Stm32_Clock_Init(336,16,4,7);
void Stm32_Clock_Init(uint32_t plln,uint32_t pllm,uint32_t pllp,uint32_t pllq)
{  
	RCC->CR|=0x00000001;		//设置HISON,开启内部高速RC振荡
	RCC->CFGR=0x00000000;		//CFGR清零 
	RCC->CR&=0xFEF6FFFF;		//HSEON,CSSON,PLLON清零 
	RCC->PLLCFGR=0x24003010;	//PLLCFGR恢复复位值 
	RCC->CR&=~(1<<18);			//HSEBYP清零,外部晶振不旁路
	RCC->CIR=0x00000000;		//禁止RCC时钟中断 
	Sys_Clock_Set(plln,pllm,pllp,pllq);//设置时钟 
	//配置向量表				  
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(1<<29,0x0);
#else   
	MY_NVIC_SetVectorTable(0,0x0);
#endif 
}


void HX717_init(uint32_t sensor_count)
// {
//     UART1_Printf("HX717 exi init %d....\r\n",sensor_count);
//     DOUT_input_mode(sensor_count);
//     SCK_init(sensor_count);
//     EXTI_init(sensor_count);
// }
{
    UART1_Printf("HX717 exi init %d....\r\n",sensor_count);
    for (int i=0; i<sensor_count; i++  ) {
        DOUT_input_mode(i);
        SCK_init(i);
    }
    EXTI_init(2); 
}

void Test(void)
{
    // uint32_t Pin_Num = 3;
    // uint8_t GPIO_MODE_OUT = 1;  // 输出IO
    // uint8_t GPIO_OTYPE_PP = 0;  //推挽输出
    // uint8_t GPIO_SPEED_100M = 2;
    // uint8_t GPIO_PUPD_PU = 1;   // 1 上拉，2 下来    
    // RCC->AHB1ENR|=1<<0;//使能PORTA时钟 
    // gpio_set(GPIOA,1 << 5,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //设置    
    HX717_init(4);
    
    while(1)
    {

        UART1_Printf("running per 2 second...%d %d %d %d\r\n", g_hx711_exi_data[0], g_hx711_exi_data[1], g_hx711_exi_data[2], g_hx711_exi_data[3]);
        my_delay_us(2000*1000);
    }
    
}
