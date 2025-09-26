// Basic scheduling functions and startup/shutdown code.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <setjmp.h> // setjmp
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // stats_update
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "board/pgm.h" // READP
#include "command.h" // shutdown
#include "sched.h" // sched_check_periodic
#include "stepper.h" // stepper_event
#include "cli.h"
#include "rtc.h"

static struct timer periodic_timer, sentinel_timer, deleted_timer;

static struct {
    struct timer *timer_list, *last_insert;
    int8_t tasks_status;
    uint8_t shutdown_status, shutdown_reason;
} SchedStatus = {.timer_list = &periodic_timer, .last_insert = &periodic_timer};


/****************************************************************
 * Timers
 ****************************************************************/

// The periodic_timer simplifies the timer code by ensuring there is
// always a timer on the timer list and that there is always a timer
// not far in the future.
static uint_fast8_t
periodic_event(struct timer *t)
{
    // Make sure the stats task runs periodically
    sched_wake_tasks();
    // Reschedule timer
    periodic_timer.waketime += timer_from_us(100000);
    sentinel_timer.waketime = periodic_timer.waketime + 0x80000000;
    return SF_RESCHEDULE;
}

static struct timer periodic_timer = {
    .func = periodic_event,
    .next = &sentinel_timer,
};

// The sentinel timer is always the last timer on timer_list - its
// presence allows the code to avoid checking for NULL while
// traversing timer_list.  Since sentinel_timer.waketime is always
// equal to (periodic_timer.waketime + 0x80000000) any added timer
// must always have a waketime less than one of these two timers.
static uint_fast8_t
sentinel_event(struct timer *t)
{
    shutdown("sentinel timer called");
}

static struct timer sentinel_timer = {
    .func = sentinel_event,
    .waketime = 0x80000000,
};

// Find position for a timer in timer_list and insert it
static void __always_inline
insert_timer(struct timer *pos, struct timer *t, uint32_t waketime)
{
    struct timer *prev;
    for (;;) {
        prev = pos;
        if (CONFIG_MACH_AVR)
            // micro optimization for AVR - reduces register pressure
            asm("" : "+r"(prev));
        pos = pos->next;
        if (timer_is_before(waketime, pos->waketime))
            break;
    }
    t->next = pos;
    prev->next = t;
}

// Schedule a function call at a supplied time.
void
sched_add_timer(struct timer *add)
{
    uint32_t waketime = add->waketime;
    irqstatus_t flag = irq_save();
    struct timer *tl = SchedStatus.timer_list;
    if (unlikely(timer_is_before(waketime, tl->waketime))) {
        // This timer is before all other scheduled timers
        if (timer_is_before(waketime, timer_read_time()))
            // try_shutdown("Timer too close");
            UART1_Printf("[sched_add_timer]Timer too close\r\n");
        if (tl == &deleted_timer)
            add->next = deleted_timer.next;
        else
            add->next = tl;
        deleted_timer.waketime = waketime;
        deleted_timer.next = add;
        SchedStatus.timer_list = &deleted_timer;
        timer_kick();
    } else {
        insert_timer(tl, add, waketime);
    }
    irq_restore(flag);
}

// The deleted timer is used when deleting an active timer.
static uint_fast8_t
deleted_event(struct timer *t)
{
    return SF_DONE;
}

static struct timer deleted_timer = {
    .func = deleted_event,
};

// Remove a timer that may be live.
void
sched_del_timer(struct timer *del)
{
    irqstatus_t flag = irq_save();
    if (SchedStatus.timer_list == del) {
        // Deleting the next active timer - replace with deleted_timer
        deleted_timer.waketime = del->waketime;
        deleted_timer.next = del->next;
        SchedStatus.timer_list = &deleted_timer;
    } else {
        // Find and remove from timer list (if present)
        struct timer *pos;
        for (pos = SchedStatus.timer_list; pos->next; pos = pos->next) {
            if (pos->next == del) {
                pos->next = del->next;
                break;
            }
        }
    }
    if (SchedStatus.last_insert == del)
        SchedStatus.last_insert = &periodic_timer;
    irq_restore(flag);
}

// Invoke the next timer - called from board hardware irq code.
unsigned int
sched_timer_dispatch(void)
{
    // Invoke timer callback
    struct timer *t = SchedStatus.timer_list;// 获取定时器列表的第一个定时器
    uint_fast8_t res;// 用于存储定时器回调的结果
    uint32_t updated_waketime;// 用于存储更新后的唤醒时间
    if (CONFIG_INLINE_STEPPER_HACK && likely(!t->func)) {
        // 如果配置了内联步进器优化，并且定时器没有设置回调函数，那么就调用步进器事件处理函数
        res = stepper_event(t);
        updated_waketime = t->waketime;// 更新唤醒时间
    } else {
        // 否则，就调用定时器的回调函数
        res = t->func(t);
        updated_waketime = t->waketime;// 更新唤醒时间
    }

    // 更新定时器列表（如果需要的话，重新调度当前定时器）
    unsigned int next_waketime = updated_waketime; // 下一个唤醒时间默认为更新后的唤醒时间
    if (unlikely(res == SF_DONE)) {
        // 如果定时器回调的结果表示定时器任务已经完成，那么就需要从定时器列表中移除当前定时器
        next_waketime = t->next->waketime;// 下一个唤醒时间为下一个定时器的唤醒时间
        SchedStatus.timer_list = t->next;// 更新定时器列表的头部为下一个定时器
        if (SchedStatus.last_insert == t)
            SchedStatus.last_insert = t->next;// 如果最后插入的定时器是当前定时器，那么就更新最后插入的定时器为下一个定时器
    } else if (!timer_is_before(updated_waketime, t->next->waketime)) {
        // 如果更新后的唤醒时间不早于下一个定时器的唤醒时间，那么就需要重新调度当前定时器
        next_waketime = t->next->waketime;// 下一个唤醒时间为下一个定时器的唤醒时间
        SchedStatus.timer_list = t->next;// 更新定时器列表的头部为下一个定时器
        struct timer *pos = SchedStatus.last_insert;// 获取最后插入的定时器
        if (timer_is_before(updated_waketime, pos->waketime))
            pos = SchedStatus.timer_list;// 如果更新后的唤醒时间早于最后插入的定时器的唤醒时间，那么就更新插入位置为定时器列表的头部
        insert_timer(pos, t, updated_waketime);// 在指定位置插入当前定时器
        SchedStatus.last_insert = t;// 更新最后插入的定时器为当前定时器
    }

    return next_waketime;// 返回下一个唤醒时间
}

// Remove all user timers
void
sched_timer_reset(void)
{
    SchedStatus.timer_list = &deleted_timer;
    deleted_timer.waketime = periodic_timer.waketime;
    deleted_timer.next = SchedStatus.last_insert = &periodic_timer;
    periodic_timer.next = &sentinel_timer;
    timer_kick();
}


/****************************************************************
 * Tasks
 ****************************************************************/

#define TS_IDLE      -1
#define TS_REQUESTED 0
#define TS_RUNNING   1

// Note that at least one task is ready to run
void
sched_wake_tasks(void)
{
    SchedStatus.tasks_status = TS_REQUESTED;
}

// Check if tasks need to be run
uint8_t
sched_tasks_busy(void)
{
    return SchedStatus.tasks_status >= TS_REQUESTED;
}

// Note that a task is ready to run
void
sched_wake_task(struct task_wake *w)
{
    sched_wake_tasks();
    writeb(&w->wake, 1);
}

// Check if a task is ready to run (as indicated by sched_wake_task)
uint8_t
sched_check_wake(struct task_wake *w)
{
    if (!readb(&w->wake))
        return 0;
    writeb(&w->wake, 0);
    return 1;
}

// Main task dispatch loop
static void
run_tasks(void)
{
    uint32_t start = timer_read_time();
    struct TimeStructure ptime;
    unsigned int timestamp_ms = 0;
    uint32_t loop = 0;
    for (;;) {
        loop++;
        // rtc_gettime(&ptime);
        // timestamp_ms = rtc_secfromdate(&ptime, DATE_BEIJING);
        // if (loop%1000 == 0){
        //     UART1_Printf("[INFO]tick:%d \r\n", timestamp_ms);
        // }

        // rtc_datefromsec(&ptime,timestamp_ms/1000,DATE_BEIJING);
        // UART1_Printf("\r\n %d/%d/%d %d:%d:%d\r\n", ptime.year, ptime.mon, ptime.date,
        //     ptime.hour, ptime.min, ptime.sec);
        cli_task();
        // Check if can sleep
        irq_poll();
        if (SchedStatus.tasks_status != TS_REQUESTED) {
            start -= timer_read_time();
            irq_disable();
            if (SchedStatus.tasks_status != TS_REQUESTED) {
                // Sleep processor (only run timers) until tasks woken
                SchedStatus.tasks_status = TS_IDLE;
                do {
                    irq_wait();
                } while (SchedStatus.tasks_status != TS_REQUESTED);
            }
            irq_enable();
            start += timer_read_time();
        }
        SchedStatus.tasks_status = TS_RUNNING;

        // Run all tasks
        extern void ctr_run_taskfuncs(void);
        ctr_run_taskfuncs();

        // Update statistics
        uint32_t cur = timer_read_time();
        stats_update(start, cur);
        start = cur;
    }
}


/****************************************************************
 * Shutdown processing
 ****************************************************************/

// Return true if the machine is in an emergency stop state
uint8_t
sched_is_shutdown(void)
{
    return !!SchedStatus.shutdown_status;
}

// Transition out of shutdown state
void
sched_clear_shutdown(void)
{
    if (!SchedStatus.shutdown_status)
        shutdown("Shutdown cleared when not shutdown");
    if (SchedStatus.shutdown_status == 2)
        // Ignore attempt to clear shutdown if still processing shutdown
        return;
    SchedStatus.shutdown_status = 0;
}

// Invoke all shutdown functions (as declared by DECL_SHUTDOWN)
static void
run_shutdown(int reason)
{
    irq_disable();
    uint32_t cur = timer_read_time();
    if (!SchedStatus.shutdown_status)
        SchedStatus.shutdown_reason = reason;
    SchedStatus.shutdown_status = 2;
    sched_timer_reset();
    extern void ctr_run_shutdownfuncs(void);
    ctr_run_shutdownfuncs();
    SchedStatus.shutdown_status = 1;
    irq_enable();

    sendf("shutdown clock=%u static_string_id=%hu", cur
          , SchedStatus.shutdown_reason);
}

// Report the last shutdown reason code
void
sched_report_shutdown(void)
{
    sendf("is_shutdown static_string_id=%hu", SchedStatus.shutdown_reason);
}

// Shutdown the machine if not already in the process of shutting down
void __always_inline
sched_try_shutdown(uint_fast8_t reason)
{
    if (!SchedStatus.shutdown_status)
        sched_shutdown(reason);
}

static jmp_buf shutdown_jmp;

// Force the machine to immediately run the shutdown handlers
void
sched_shutdown(uint_fast8_t reason)
{
    irq_disable();
    longjmp(shutdown_jmp, reason);
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main loop of program
void
sched_main(void)
{
    extern void ctr_run_initfuncs(void);
    ctr_run_initfuncs();
    
    UART1_Printf("ctr_run_initfuncs done\r\n");
    sendf("starting");

    irq_disable();
    int ret = setjmp(shutdown_jmp);
    if (ret)
        run_shutdown(ret);
    irq_enable();
 
    run_tasks();
}
