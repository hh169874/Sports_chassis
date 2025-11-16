#include "retarget.h"

/* 告知连接器不从C库链接使用半主机的函数 */
#pragma import(__use_no_semihosting)

/* 定义USART端口，用于注册重定向的串口 */
static UART_HandleTypeDef *sg_hUart;

/**
 * @brief 注册重定向串口
 * @param usartx 需要重定向输入输出的串口句柄
 */
void RetargetInit(UART_HandleTypeDef *huart) {
    /* 注册串口 */
    sg_hUart = huart;
    /* Disable I/O buffering for STDOUT stream, so that
     * chars are sent out as soon as they are printed. */
    setvbuf(stdout, NULL, _IONBF, 0);
    /* Disable I/O buffering for STDIN stream, so that
     * chars are received in as soon as they are scanned. */
    setvbuf(stdin, NULL, _IONBF, 0);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
    /* 发送一个字节数据到串口 */
    /* 很简单，直接调用 HAL 库中的 串口发送数据函数 */
    HAL_UART_Transmit(sg_hUart, (uint8_t *)&ch, 1, 0xFFFF);
    /* 返回发送的字符 */
    return ch;
}

/**
  * @brief  Retargets the C library scanf, getchar function to the USART.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE {
    /* 用于接收数据 */
    uint8_t ch;
    /* 调用 HAL 库中的接收函数 */
    HAL_UART_Receive(sg_hUart, (uint8_t *) &ch, 1, 0xFFFF);
    /* 直接返回接收到的字符 */
    return (int) ch;
}