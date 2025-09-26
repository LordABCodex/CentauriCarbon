
#ifndef __CLI_H__
#define __CLI_H__

typedef void (*cli_handle)(int argc, char *argv[]);

/* 命令结构 */
struct _ST_CLI_COMMAND_ {
  char *name;
  cli_handle hook;
  char *summary;
};

#define CLI_COMMAND_BUFFER_SIZE (400) /* 输入的命令和参数长度 */
#define CLI_COMMAND_PARA_NUMBER (15) /* 命令支持的输入的参数的个数 */
#define CLI_COMMAND_PARA_LENGTH (200) /* 命令支持的输入的参数的长度 */

#define MODE_CLI (0)
#define MODE_XMODEM (1)

#define RET_SUCCESS (0)
#define RET_FAIL (1)
#define RET_NULL (2)
#define RET_OVERFLOW (3)
#define RET_DONE (4)
#define RET_PROCESSING (5)
#define RET_ERROR (6)
#define RET_PARA (7)
#define RET_BUSY (8)
#define RET_INIT (9)
#define RET_TEST (10)

typedef enum { FALSE = 0, TRUE = !FALSE } bool;

#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef void (*upgradehandle)(void);

/* 命令行控制结构体 */
struct st_cli {
  int mode; /* 0:命令行/1:xmodem模式*/

  char aucCommandBuffer[CLI_COMMAND_BUFFER_SIZE]; /* 输入命令字符buffer   */
  int ulCommandCharCnt; /* 输入命令字符个数     */

  char *aCliPara[CLI_COMMAND_PARA_NUMBER]; /* 解析出的参数         */
  int ulParaNumber;                        /* 解析出的参数的个数   */
};

/* 串口buffer */
#define USART_RX_BUFFER_SIZE (512)

/* 串口接收buffer */
struct st_usart_rx_q {
  int number;
  int readptr;
  int writeptr;
  char buffer[USART_RX_BUFFER_SIZE];

  int traceflag;
};

/* 全局变量声明 */
extern struct st_cli g_stcli;

extern struct st_usart_rx_q g_stusart1buffer;

/* 全局函数声明 */
extern void cli_init(void);
extern void cli_para_init(void);
extern int cli_getpara(void);
extern int cli_analyze(void);
extern void cli_proc(void *pdata);
extern void cli_task(void);
extern int usart1_pend(char *pchar);
extern void hx711s_cli(int argc, char *argv[]);

#endif /* __CLI_H__ */
