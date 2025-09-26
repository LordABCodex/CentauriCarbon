// Timer based on ARM Cortex-M3/M4 SysTick and DWT logic
//
// Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "armcm_boot.h" // DECL_ARMCM_IRQ
#include "board/internal.h" // SysTick
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "sched.h" // sched_timer_dispatch
#define TIMER_READ_TIME_DEBUG 0
#if TIMER_READ_TIME_DEBUG == 1
static uint32_t timer_read_time_debug = 0;
#endif
DECL_CONSTANT("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

// Return the number of clock ticks for a given number of microseconds
uint32_t
timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
uint8_t
timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}

// Set the next irq time
static void
timer_set_diff(uint32_t value)
{
    SysTick->LOAD = value;
    SysTick->VAL = 0;
    SysTick->LOAD = 0;
}

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
    #if TIMER_READ_TIME_DEBUG == 1
    timer_read_time_debug = DWT->CYCCNT;
    return timer_read_time_debug;
    #else
    return DWT->CYCCNT;
    #endif
    
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
}

// Implement simple early-boot delay mechanism
void
udelay(uint32_t usecs)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t end = timer_read_time() + timer_from_us(usecs);
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Dummy timer to avoid scheduling a SysTick irq greater than 0xffffff
static uint_fast8_t
timer_wrap_event(struct timer *t)
{
    t->waketime += 0xffffff;
    return SF_RESCHEDULE;
}
static struct timer wrap_timer = {
    .func = timer_wrap_event,
    .waketime = 0xffffff,
};
void
timer_reset(void)
{
    if (timer_from_us(100000) <= 0xffffff)
        // Timer in sched.c already ensures SysTick wont overflow
        return;
    sched_add_timer(&wrap_timer);
}
DECL_SHUTDOWN(timer_reset);

void
timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    // Schedule a recurring timer on fast cpus
    timer_reset();

    // Enable SysTick
    irqstatus_t flag = irq_save();
    NVIC_SetPriority(SysTick_IRQn, 2);
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
                     | SysTick_CTRL_ENABLE_Msk);
    timer_kick();
    irq_restore(flag);
}
DECL_INIT(timer_init);

static uint32_t timer_repeat_until;
#define TIMER_IDLE_REPEAT_TICKS timer_from_us(500)
#define TIMER_REPEAT_TICKS timer_from_us(100)

#define TIMER_MIN_TRY_TICKS timer_from_us(2)
#define TIMER_DEFER_REPEAT_TICKS timer_from_us(5)

// Invoke timers
static uint32_t
timer_dispatch_many(void)
{
    uint32_t tru = timer_repeat_until;// 获取定时器重复的截止时间
    for (;;) {// 无限循环，直到函数返回
        // 运行下一个软件定时器
        uint32_t next = sched_timer_dispatch();// 调度并执行下一个软件定时器任务

        uint32_t now = timer_read_time();// 读取当前的时间
        int32_t diff = next - now;// 计算下一个定时器任务的预定时间与当前时间的差值
        if (diff > (int32_t)TIMER_MIN_TRY_TICKS)
             // 如果下一个定时器任务的预定时间距离现在还有一段时间，那么就返回这个差值
            return diff;

        if (unlikely(timer_is_before(tru, now))) {
            // 如果定时器的重复截止时间已经过去了，那么就需要检查是否有太多的重复定时器
            if (diff < (int32_t)(-timer_from_us(10000000))){
                // 如果下一个定时器任务的预定时间已经过去了，那么就尝试关闭系统
                try_shutdown("Rescheduled timer in the past"); 
            }
            if (sched_tasks_busy()) {
                // 如果还有其他任务正在执行，那么就更新定时器的重复截止时间，并返回一个特殊的值
                timer_repeat_until = now + TIMER_REPEAT_TICKS;
                return TIMER_DEFER_REPEAT_TICKS;
            }
            // 如果没有其他任务正在执行，那么就更新定时器的重复截止时间为当前时间加上一个空闲重复的时间间隔
            timer_repeat_until = tru = now + TIMER_IDLE_REPEAT_TICKS;
        }

        // Next timer in the past or near future - wait for it to be ready
        irq_enable();
        while (unlikely(diff > 0))
            diff = next - timer_read_time();
        irq_disable();
    }
}

// IRQ handler
void __visible __aligned(16) // aligning helps stabilize perf benchmarks
SysTick_Handler(void)
{
    irq_disable();
    uint32_t diff = timer_dispatch_many();
    timer_set_diff(diff);
    irq_enable();
}
DECL_ARMCM_IRQ(SysTick_Handler, SysTick_IRQn);

// Make sure timer_repeat_until doesn't wrap 32bit comparisons
void
timer_task(void)
{
    uint32_t now = timer_read_time();
    irq_disable();
    if (timer_is_before(timer_repeat_until, now))
        timer_repeat_until = now;
    irq_enable();
}
DECL_TASK(timer_task);
