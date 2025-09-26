
#include "cli.h"

#include <stdio.h>
#include <string.h>

#include "basecmd.h"         // oid_alloc
#include "board/gpio.h"      // struct gpio_in
#include "board/internal.h"  // gpio_peripheral
#include "board/irq.h"       // irq_disable
#include "board/misc.h"      // timer_from_us
#include "command.h"         // DECL_COMMAND
#include "sched.h"           // struct timer

/* 输入命令解析控制控制 */
struct st_cli g_stcli = {0};

struct st_usart_rx_q g_stusart1buffer;

/* 命令函数 */
void test(int argc, char* argv[]);
void version(int argc, char* argv[]);
void cli_dm(int argc, char* argv[]);
void cli_dr(int argc, char* argv[]);
void cli_mm(int argc, char* argv[]);
void cli_mr(int argc, char* argv[]);
void reset_cli(int argc, char* argv[]);
void goto_cli(int argc, char* argv[]);
void math_cli(int argc, char* argv[]);

/* 命令列表 */
struct _ST_CLI_COMMAND_ g_stCommand[] = {
    /*
    命令             函数体             命令简介        */
    {"test", test, "test as you want"},
    {"dm", cli_dm, "read memery"},
    {"dr", cli_dr, "read register"},
    {"mm", cli_mm, "write memery"},
    {"mr", cli_mr, "write register"},
    // {"math", math_cli, "math testing"},
    {"reset", reset_cli, "reset"},
    {"goto", goto_cli, "goto xx"},
    {"hx711s", hx711s_cli, "hx711s sensor check"},
    {0, 0, 0}};

/**
 * @description: 复位函数
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void reset_cli(int argc, char* argv[]) {
  UART1_Printf("\r\n\r\nSystem will reset ... \r\n");
  //   OSTimeDly(10);
  NVIC_SystemReset();
}

/**
 * @description: 跳转接口
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void goto_cli(int argc, char* argv[]) {
  upgradehandle pfunc;

  pfunc = (upgradehandle)strtoul(argv[0], NULL, 16);
  UART1_Printf("\r\nGoto addr:0x%08X", (unsigned int)pfunc);
  //   OSTimeDly(10);
  pfunc();
}

/**
 * @description: 数学函数
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
// void math_cli(int argc, char* argv[]) {
//   double a, b, c;
//   double temp;

//   if (0 == strcmp("-abc", argv[0])) {
//     if (0 == strcmp("c", argv[1])) {
//       a = atof(argv[2]);
//       b = atof(argv[3]);
//       UART1_Printf("\r\na:%.4f", a);
//       UART1_Printf("\r\nb:%.4f", b);
//       UART1_Printf("\r\nc:%.4f", sqrt(a * a + b * b));
//     } else if (0 == strcmp("a", argv[1])) {
//       c = atof(argv[2]);
//       b = atof(argv[3]);
//       UART1_Printf("\r\nc:%.4f", c);
//       UART1_Printf("\r\nb:%.4f", b);
//       UART1_Printf("\r\na:%.4f", sqrt(c * c - b * b));
//     } else if (0 == strcmp("b", argv[1])) {
//       c = atof(argv[2]);
//       a = atof(argv[3]);
//       UART1_Printf("\r\nc:%.4f", c);
//       UART1_Printf("\r\na:%.4f", a);
//       UART1_Printf("\r\nb:%.4f", sqrt(c * c - a * a));
//     } else {
//       UART1_Printf("\r\nmath_cli -abc para err.");
//     }
//   } else if (0 == strcmp("-sin", argv[0])) {
//     temp = atof(argv[1]) * 3.1415926 / 180.0;
//     UART1_Printf("\r\nsin(%.4f) = %.4f;", atof(argv[1]), sin(temp));
//   } else if (0 == strcmp("-cos", argv[0])) {
//     temp = atof(argv[1]) * 3.1415926 / 180.0;
//     UART1_Printf("\r\ncos(%.4f) = %.4f;", atof(argv[1]), cos(temp));
//   } else if (0 == strcmp("-tan", argv[0])) {
//     temp = atof(argv[1]) * 3.1415926 / 180.0;
//     UART1_Printf("\r\ntan(%.4f) = %.4f;", atof(argv[1]), tan(temp));
//   } else if (0 == strcmp("-asin", argv[0])) {
//     UART1_Printf("\r\nasin(%.4f) = %.4f;", atof(argv[1]),
//                  asin(atof(argv[1])) * 180.0 / 3.1415926);
//   } else if (0 == strcmp("-acos", argv[0])) {
//     UART1_Printf("\r\nacos(%.4f) = %.4f;", atof(argv[1]),
//                  acos(atof(argv[1])) * 180.0 / 3.1415926);
//   } else if (0 == strcmp("-atan", argv[0])) {
//     UART1_Printf("\r\natan(%.4f) = %.4f;", atof(argv[1]),
//                  atan(atof(argv[1])) * 180.0 / 3.1415926);
//   } else {
//     UART1_Printf("\r\nmath_cli para err.");
//   }
// }

/**
 * @description: 测试函数
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void test(int argc, char* argv[]) {
  int i;

  for (i = 0; i < argc; i++) {
    UART1_Printf("\r\n%d:%s", i, argv[i]);
  }
}

/**
 * @description: 打印memery内容
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void cli_dm(int argc, char* argv[]) {
  int32_t addr;
  int32_t len;
  int32_t i;

  /* 打印该命令的使用说明 */
  if ((2 != argc) || (0 == strcmp("?", argv[0]))) {
    UART1_Printf("\r\nDESCRIPTION:\r\n    View memory data by hexadecimal");
    UART1_Printf("\r\nSYNOPSIS:\r\n    dm [addr] [len]");
    UART1_Printf("\r\nEXAMPLES:\r\n    dm 0x20000000 0x1000");
    return;
  }

  if (2 == argc) {
    addr = strtoul(argv[0], NULL, 16);
    len = strtoul(argv[1], NULL, 16);
    for (i = 0; i < len; i++) {
      if (!(i % 16)) {
        UART1_Printf("\r\n0x%08X:", addr + i);
        // OSTimeDly(10);
      }

      UART1_Printf(" %02X", *(uint8_t*)(addr + i));
    }
  }
}

/**
 * @description: 打印register内容
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void cli_dr(int argc, char* argv[]) {
  int32_t addr;
  int32_t len;
  int32_t i;

  /* 打印该命令的使用说明 */
  if ((2 != argc) || (0 == strcmp("?", argv[0]))) {
    UART1_Printf("\r\nDESCRIPTION:\r\n    View register data by hexadecimal");
    UART1_Printf("\r\nSYNOPSIS:\r\n    dr [addr] [len]");
    UART1_Printf("\r\nEXAMPLES:\r\n    dr 0x20000000 0x10");
    return;
  }

  if (2 == argc) {
    addr = strtoul(argv[0], NULL, 16);
    len = strtoul(argv[1], NULL, 16);
    for (i = 0; i < len; i++) {
      if (!(i % 4)) {
        UART1_Printf("\r\n0x%08X:", addr + i * 4);
        // OSTimeDly(10);
      }

      UART1_Printf(" %08X", *(int32_t*)(addr + i * 4));
    }
  }
}

/**
 * @description: 修改memery内容
 * @author: your name
 * @return {*}
 */

void cli_mm(int argc, char* argv[]) {
  int32_t addr;
  int32_t data;
  int32_t len;
  int32_t i;

  /* 打印该命令的使用说明 */
  if ((3 != argc) || (0 == strcmp("?", argv[0]))) {
    UART1_Printf("\r\nDESCRIPTION:\r\n    Write memery data by hexadecimal");
    UART1_Printf("\r\nSYNOPSIS:\r\n    mm [addr] [len] [data]");
    UART1_Printf("\r\nEXAMPLES:\r\n    mm 0x20000000 0x1000 0xAA");
    return;
  }

  if (3 == argc) {
    addr = strtoul(argv[0], NULL, 16);
    len = strtoul(argv[1], NULL, 16);
    data = strtoul(argv[2], NULL, 16);
    for (i = 0; i < len; i++) {
      *(uint8_t*)(addr + i) = data;
    }
    UART1_Printf("\r\nWrite 0x%02X at 0x%08X-0x%08X", data, addr, addr + len);
  }
}

/**
 * @description: 修改register内容
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void cli_mr(int argc, char* argv[]) {
  int32_t addr;
  int32_t data;

  /* 打印该命令的使用说明 */
  if ((2 != argc) || (0 == strcmp("?", argv[0]))) {
    UART1_Printf("\r\nDESCRIPTION:\r\n    Write register data by hexadecimal");
    UART1_Printf("\r\nSYNOPSIS:\r\n    mr [addr] [data]");
    UART1_Printf("\r\nEXAMPLES:\r\n    mr 0x20000000 0xAABBCCDD");
    return;
  }

  if (2 == argc) {
    addr = strtoul(argv[0], NULL, 16);
    data = (int)strtoul(argv[1], NULL, 16);
    *(int32_t*)addr = data;
    UART1_Printf("\r\nWrite 0x%08X at 0x%08X", data, addr);
  }
}

/**
 * @description: 获取参数，以用户输入的enter为结束
 * @author: your name
 * @return {*}
 */
int cli_getpara(void) {
  char temp;
  int ret;

  irq_disable();
  for (;;) {
    /* 阻塞式获取字符 */
    ret = usart1_pend(&temp);
    if (RET_SUCCESS != ret) {
      // UART1_Printf("cli_getpara RET_SUCCESS");
      return ret;
    }
    /* enter */
    if ('\r' == temp) {
      return RET_DONE;
    }
    /* 删除字符 */
    else if ('\b' == temp) {
      if (g_stcli.ulCommandCharCnt) {
        g_stcli.ulCommandCharCnt--;
        UART1_Printf("\b \b");
      }
    } else {
      /* 快捷键复位 */
      if (!g_stcli.ulCommandCharCnt) {
        if ('`' == temp) {
          reset_cli(0, 0);
          cli_para_init();
          continue;
        } else if ('~' == temp) {
          // xmodem_cli(0, 0);
          cli_para_init();
          return RET_DONE;
        }
      }

      /* 输入字符超过限制 */
      if (CLI_COMMAND_BUFFER_SIZE <= g_stcli.ulCommandCharCnt) {
        UART1_Printf("\r\nCLI char number exceed %d.", CLI_COMMAND_BUFFER_SIZE);
        g_stcli.ulCommandCharCnt = 0;
        return RET_DONE;
      }
      UART1_Printf("%c", temp);
      g_stcli.aucCommandBuffer[g_stcli.ulCommandCharCnt] = temp;
      g_stcli.ulCommandCharCnt++;
    }
  }
  irq_enable();
}

/**
 * @description: 串口解析参数函数
 * @author: your name
 * @return {*}
 */
int cli_analyze(void) {
  int ulAnalyzeState = FALSE;  // FALSE: 开始解析参数状态, TRUE: 参数解析中状态;
  int ulParaLen = 0;
  int i = 0;

  // UART1_Printf("cli_analyze");
  for (i = 0; i < g_stcli.ulCommandCharCnt; i++) {
    /* 回车和换行忽略 */
    if (('\n' == g_stcli.aucCommandBuffer[i]) ||
        ('\r' == g_stcli.aucCommandBuffer[i])) {
      continue;
    }

    /* SPACE键处理 */
    if (' ' == g_stcli.aucCommandBuffer[i]) {
      /* 参数解析中状态要切换到开始解析参数状态 */
      if (TRUE == ulAnalyzeState) {
        g_stcli.aCliPara[g_stcli.ulParaNumber - 1][ulParaLen] = '\0';
        ulAnalyzeState = FALSE;
      }
      continue;
    }

    /* 新的参数解析的开始 */
    if (FALSE == ulAnalyzeState) {
      ulParaLen = 0;
      g_stcli.ulParaNumber++;

      /* 参数超过限制 */
      if (CLI_COMMAND_PARA_NUMBER < g_stcli.ulParaNumber) {
        /* 打印参数个数超过限制 */
        UART1_Printf("\r\nCLI para number exceed %d.", CLI_COMMAND_PARA_NUMBER);
        return RET_FAIL;
      }
    }

    ulAnalyzeState = TRUE;

    g_stcli.aCliPara[g_stcli.ulParaNumber - 1][ulParaLen] =
        g_stcli.aucCommandBuffer[i];
    ulParaLen++;

    /* 参数长度限制, 留个空间存放'\0' */
    if ((CLI_COMMAND_PARA_LENGTH - 1) <= ulParaLen) {
      /* 打印参数的长度超过限制 */
      UART1_Printf("\r\nCLI para length exceed %d.", CLI_COMMAND_PARA_LENGTH);
      return RET_FAIL;
    }
  }

  /* 补充最后一个参数的个数 */
  if (TRUE == ulAnalyzeState) {
    g_stcli.aCliPara[g_stcli.ulParaNumber - 1][ulParaLen] = '\0';
  }

  return RET_SUCCESS;
}

/**
 * @description: 命令行命令执行函数
 * @author: your name
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
void cli_execute(int argc, char* argv[]) {
  int i = 0;

  /* 参数为空 */
  if (!argc) {
    return;
  }

  /* 命令查询 */
  if ((0 == strcmp("?", argv[0])) || (0 == strcmp("help", argv[0]))) {
    UART1_Printf("\r\nCommand\t\tDescribe");
    while (g_stCommand[i].name) {
      /* 命令行对齐, 8个字符位置等于一个TAB */
      if (8 <= strlen(g_stCommand[i].name)) {
        UART1_Printf("\r\n%s\t\"%s\"", g_stCommand[i].name,
                     g_stCommand[i].summary);
      } else {
        UART1_Printf("\r\n%s\t\t\"%s\"", g_stCommand[i].name,
                     g_stCommand[i].summary);
      }
      i++;
    }
    return;
  }

  while (g_stCommand[i].name) {
    /* 匹配命令 */
    if (0 == strcmp(g_stCommand[i].name, argv[0])) {
      g_stCommand[i].hook(argc - 1, &argv[1]);
      return;
    }
    i++;
  }

  UART1_Printf("\r\n%s:command not found", argv[0]);
}

/**
 * @description: 命令主体任务
          1、阻塞式获取用户输入的命令；
          2、解析出参数，命令任务是特殊的参数；
          3、执行命令；
          4、打印提示符，初始化控制数据结构；
 * @author: your name
 * @return {*}
 */
void cli_main(void) {
  /* 从串口获取用户命令参数, 以输入回车为完成标志 */
  if (RET_DONE == cli_getpara()) {
    /* 解析参数 */
    if (RET_SUCCESS == cli_analyze()) {
      /* 执行命令 */
      cli_execute(g_stcli.ulParaNumber, g_stcli.aCliPara);
    }
    /* 结束处理 */
    cli_para_init();
    UART1_Printf("\r\n->");
  }
}

/**
 * @description: 命令行任务
 * @author: your name
 * @param {void*} pdata
 * @return {*}
 */
void cli_proc(void* pdata) {
  cli_init();
  UART1_Printf("\r\n->");

  for (;;) {
    /* cli模式 */
    if (MODE_CLI == g_stcli.mode) {
      cli_main();
    }
    /* xmodem模式 */
    else {
      // xmodem_main();
    }
  }
}

/**
 * @description: 命令行输入字符解析等控制数据初始化
 * @author: your name
 * @return {*}
 */
void cli_para_init(void) {
  int i;

  g_stcli.ulCommandCharCnt = 0;
  for (i = 0; i < CLI_COMMAND_BUFFER_SIZE; i++) {
    g_stcli.aucCommandBuffer[i] = 0;
  }

  g_stcli.ulParaNumber = 0;
  for (i = 0; i < CLI_COMMAND_PARA_NUMBER; i++) {
    memset(g_stcli.aCliPara[i], 0, CLI_COMMAND_PARA_LENGTH);
  }
}

/**
 * @description: 命令行初始化
 * @author: your name
 * @return {*}
 */
void cli_init(void) {
  int i;

  /* 默认命令行模式 */
  g_stcli.mode = MODE_CLI;
  g_stusart1buffer.traceflag = 0;

  for (i = 0; i < CLI_COMMAND_PARA_NUMBER; i++) {
    g_stcli.aCliPara[i] = (char*)malloc(CLI_COMMAND_PARA_LENGTH);
  }
  cli_para_init();
}

// CLI采样周期：us
#define CLI_SAMPLE_PERIOD_MS 13000  // CLI  最大采样率：80SPS
#define CLI_SAMPLE_PERIOD_TICKS \
  (CONFIG_CLOCK_FREQ / 1000000 * CLI_SAMPLE_PERIOD_MS)
#define CLI_SAMPLE_REST_TICKS (CLI_SAMPLE_PERIOD_TICKS)
#define SYS_TICK_MS 1000000 / CONFIG_CLOCK_FREQ

struct cli {
  struct timer cli_timer;
  uint32_t oid;
  uint8_t flags;
  uint32_t rest_ticks;
};

struct task_wake cli_wake;

static uint_fast8_t cli_sample_event(struct timer* t) {
  sched_wake_task(&cli_wake);
  struct cli* c = container_of(t, struct cli, cli_timer);

  c->cli_timer.waketime += c->rest_ticks;

  if (c->flags) {
    return SF_RESCHEDULE;
  }
  return SF_DONE;
}

void command_config_cli(uint32_t* args) {
  struct cli* c = oid_alloc(args[0], command_config_cli, sizeof(*c));
  c->oid = args[0];
  c->rest_ticks = CLI_SAMPLE_REST_TICKS;
  c->cli_timer.func = cli_sample_event;
  c->flags = 0;
  UART1_Printf("command_config_cli, sample period:%d ms\n",
               CLI_SAMPLE_PERIOD_MS);
}
DECL_COMMAND(command_config_cli, "config_cli oid=%c ");

void command_start_cli(uint32_t* args) {
  uint8_t oid = args[0];
  struct cli* c = oid_lookup(oid, command_config_cli);
  c->flags = 1;
  sched_del_timer(&c->cli_timer);
  irq_disable();
  c->cli_timer.waketime = timer_read_time() + c->rest_ticks;
  sched_add_timer(&c->cli_timer);
  irq_enable();
}
DECL_COMMAND(command_start_cli, "start_cli oid=%c");

void cli_task(void) {
  // uint8_t oid;
  // struct cli* c;

  // if (!sched_check_wake(&cli_wake)) {
  //   return;
  // }

  // foreach_oid(oid, c, command_config_cli)
  {
    if (MODE_CLI == g_stcli.mode) {
      cli_main();
    }
    /* xmodem模式 */
    else {
      // xmodem_main();
    }
  }
}
DECL_TASK(cli_task);

void cli_shutdown(void) {
  uint8_t oid;
  struct cli* c;
  foreach_oid(oid, c, command_config_cli) { c->flags = 0; }
  return;
}
DECL_SHUTDOWN(cli_shutdown);

/**
 * @description: 环形队列投递字符
 * @author: your name
 * @param {char} *pchar
 * @return {*}
 */
int usart1_pend(char* pchar) {
  // NVIC_SETPRIMASK();
  /* 队列为空 */
  if (g_stusart1buffer.number == 0) {
    // NVIC_RESETPRIMASK();
    return RET_NULL;
  }

  *pchar = g_stusart1buffer.buffer[g_stusart1buffer.readptr];
  g_stusart1buffer.readptr++;
  /* 翻转处理 */
  if (USART_RX_BUFFER_SIZE <= g_stusart1buffer.readptr) {
    g_stusart1buffer.readptr = 0;
  }

  g_stusart1buffer.number--;
  // NVIC_RESETPRIMASK();
  return RET_SUCCESS;
}

/**
 * @description: 环形队列投递字符
 * @author: your name
 * @param {char} temp
 * @return {*}
 */
int usart1_post(char temp) {
  /* 维测报文跟踪 */
  if (g_stusart1buffer.traceflag) {
    UART1_Printf("\r\nUSART1 RX: %c", temp);
    // return RET_TEST;
  }

  /* 接收字符队列满判断 */
  if (USART_RX_BUFFER_SIZE <= g_stusart1buffer.number) {
    // OS_EXIT_CRITICAL();
    UART1_Printf("\r\nERR: usart1_post overflow");
    return RET_OVERFLOW;
  }

  g_stusart1buffer.buffer[g_stusart1buffer.writeptr] = temp;
  g_stusart1buffer.writeptr++;
  /* 翻转处理 */
  if (USART_RX_BUFFER_SIZE <= g_stusart1buffer.writeptr) {
    g_stusart1buffer.writeptr = 0;
  }
  g_stusart1buffer.number++;
  return RET_SUCCESS;
}