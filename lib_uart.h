//------------------------------------------------------------------------------
/**
 * @file lib_uart.h
 * @author charles-park (charles.park@hardkernel.com)
 * @brief UART control library for ODROID-JIG.
 * @version 0.2
 * @date 2023-10-05
 *
 * @package apt install minicom
 *
 * @copyright Copyright (c) 2022
 *
 */
//------------------------------------------------------------------------------
#ifndef __LIB_UART_H__
#define __LIB_UART_H__

//------------------------------------------------------------------------------
#include <errno.h>      // Error integer and strerror() function
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <pthread.h>
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()

//------------------------------------------------------------------------------
#define DEFAULT_QUEUE_SIZE      1024

//------------------------------------------------------------------------------
typedef struct queue__t {
    unsigned int    sp;
    unsigned int    ep;
    unsigned int    size;
    unsigned char   *buf;
}   queue_t;

typedef struct protocol_variable__t {
	unsigned int    p_sp;
	unsigned int    p_ep;
	unsigned int    size;
	unsigned char   open;
	unsigned char   pass;
	unsigned char   *buf;
}   ptc_var_t;

typedef struct protocol_function__t {
    ptc_var_t   var;
    int         (*pcheck)(ptc_var_t *p);
    int         (*pcatch)(ptc_var_t *p);
}   ptc_func_t;

//------------------------------------------------------------------------------
typedef struct uart__t {
    int             fd;
    unsigned char   pcnt;
	ptc_func_t      *p;
    pthread_t       rx_thread, tx_thread;
    queue_t         tx_q, rx_q;
}   uart_t;

//------------------------------------------------------------------------------
extern void    ptc_set_status  (uart_t *ptc_grp, unsigned char ptc_num, unsigned char status);
extern void    ptc_q           (uart_t *ptc_grp, unsigned char ptc_num, unsigned char idata);
extern void    ptc_event       (uart_t *ptc_grp, unsigned char idata);
extern int     ptc_func_init   (uart_t *ptc_grp, unsigned char ptc_num, unsigned char ptc_size,
                                int (*chk_func)(ptc_var_t *var), int (*cat_func)(ptc_var_t *var));
extern int     ptc_grp_init    (uart_t *ptc_grp, unsigned char ptc_count);
extern void    ptc_grp_close   (uart_t *ptc_grp);

//------------------------------------------------------------------------------
extern int     uart_write      (uart_t *puart, unsigned char *w_data, int w_size);
extern int     uart_read       (uart_t *puart, unsigned char *r_data, int r_size);
extern uart_t  *uart_init      (const char *dev_name, int baud);
extern void    uart_close      (uart_t *ptc_grp);

//------------------------------------------------------------------------------
#endif  // #define __LIB_UART_H__

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
