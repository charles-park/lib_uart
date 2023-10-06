//------------------------------------------------------------------------------
/**
 * @file lib_uart.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//------------------------------------------------------------------------------
// Linux headers
//------------------------------------------------------------------------------
#include "lib_uart.h"

//------------------------------------------------------------------------------
static speed_t  baudrate    (int baud);
static int      queue_get   (queue_t *q, unsigned char *d);
static int      queue_put   (queue_t *q, unsigned char *d);

//------------------------------------------------------------------------------
void    ptc_set_status  (uart_t *ptc_grp, unsigned char ptc_num, unsigned char status);
void    ptc_q           (uart_t *ptc_grp, unsigned char ptc_num, unsigned char idata);
void    ptc_event       (uart_t *ptc_grp, unsigned char idata);
int     ptc_func_init   (uart_t *ptc_grp, unsigned char ptc_num, unsigned char ptc_size,
                                int (*chk_func)(ptc_var_t *var), int (*cat_func)(ptc_var_t *var));
int     ptc_grp_init    (uart_t *ptc_grp, unsigned char ptc_count);
void    ptc_grp_close   (uart_t *ptc_grp);

//------------------------------------------------------------------------------
int     uart_write      (uart_t *puart, unsigned char *w_data, int w_size);
int     uart_read       (uart_t *puart, unsigned char *r_data, int r_size);
uart_t  *uart_init      (const char *dev_name, int baud);
void    uart_close      (uart_t *ptc_grp);

//------------------------------------------------------------------------------
pthread_mutex_t mutex_get = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_put = PTHREAD_MUTEX_INITIALIZER;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static speed_t baudrate (int baud)
{
    switch (baud) {
        case 9600:      return B9600;
        case 19200:     return B19200;
        case 38400:     return B38400;
        case 57600:     return B57600;
        case 115200:    return B115200;
        case 230400:    return B230400;
        case 460800:    return B460800;
        case 500000:    return B500000;
        case 576000:    return B576000;
        case 921600:    return B921600;
        case 1000000:   return B1000000;
        case 1152000:   return B1152000;
        case 1500000:   return B1500000;
        case 2000000:   return B2000000;
        case 2500000:   return B2500000;
        case 3000000:   return B3000000;
        case 3500000:   return B3500000;
        case 4000000:   return B4000000;
        default:
            return B115200;
    }
}

//------------------------------------------------------------------------------
static int queue_put (queue_t *q, unsigned char *d)
{
    pthread_mutex_lock  (&mutex_put);
    q->buf[q->ep++] = *d;
    pthread_mutex_unlock(&mutex_put);

    if (q->ep >= q->size)   q->ep = 0;

    // queue overflow
    if (q->ep == q->sp) {
        q->sp++;
        return 0;
    }
    return  1;
}

//------------------------------------------------------------------------------
static int queue_get (queue_t *q, unsigned char *d)
{
    if (q->ep != q->sp) {
        pthread_mutex_lock  (&mutex_get);
        *d = q->buf[q->sp++];
        pthread_mutex_unlock(&mutex_get);
        if (q->sp >= q->size)   q->sp = 0;
        return  1;
    }
    // queue empty
    return 0;
}

//------------------------------------------------------------------------------
void *rx_thread_func (void *arg)
{
    unsigned char d;
    uart_t *puart = (uart_t *)arg;

    while(1) {
        if (read (puart->fd, &d, 1) > 0)  queue_put (&puart->rx_q, &d);
        usleep(100);
    }
}

//------------------------------------------------------------------------------
void *tx_thread_func (void *arg)
{
    unsigned char d;
    uart_t *puart = (uart_t *)arg;

    while(1) {
        if (queue_get(&puart->tx_q, &d))  write (puart->fd, &d, 1);
        usleep(100);
    }
}

//------------------------------------------------------------------------------
//   Protocol Open & Close Function
//------------------------------------------------------------------------------
void ptc_set_status (uart_t *puart, unsigned char ptc_num, unsigned char status)
{
    puart->p[ptc_num].var.open = status;
}

//------------------------------------------------------------------------------
// data save to protocol q
//------------------------------------------------------------------------------
void ptc_q (uart_t *puart, unsigned char ptc_num, unsigned char idata)
{
    puart->p[ptc_num].var.p_ep %= puart->p[ptc_num].var.size;
    if (puart->p[ptc_num].var.p_ep == puart->p[ptc_num].var.p_sp)
    {
        puart->p[ptc_num].var.p_sp++;
        puart->p[ptc_num].var.p_sp %= puart->p[ptc_num].var.size;
    }
    puart->p[ptc_num].var.buf[puart->p[ptc_num].var.p_ep++] = idata;
}

//------------------------------------------------------------------------------
//   Protocol check & data atch fron Q Buffer
//------------------------------------------------------------------------------
void ptc_event (uart_t *puart, unsigned char idata)
{
    unsigned char ptc_pos;

    for (ptc_pos = 0; ptc_pos < puart->pcnt; ptc_pos++)	{
        if (puart->p[ptc_pos].var.open)	{
            ptc_q (puart, ptc_pos, idata);
            if (puart->p[ptc_pos].pcheck (&puart->p[ptc_pos].var))	{
                if (puart->p[ptc_pos].pcatch (&puart->p[ptc_pos].var))	{
                    puart->p[ptc_pos].var.pass = 1;
                    ptc_set_status (puart, ptc_pos, 0);
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
//   UART Protocol Initiliaze Function
//------------------------------------------------------------------------------
int ptc_func_init (uart_t *puart, unsigned char ptc_num, unsigned char ptc_size,
                                int (*chk_func)(ptc_var_t *var), int (*cat_func)(ptc_var_t *var))
{
    puart->p[ptc_num].var.p_ep = 0;
    puart->p[ptc_num].var.p_sp = 0;
    puart->p[ptc_num].var.open = 1;
    puart->p[ptc_num].var.pass = 0;

    puart->p[ptc_num].var.size = ptc_size;
    puart->p[ptc_num].var.buf  = (unsigned char *)(malloc(sizeof(unsigned char) * ptc_size));
    puart->p[ptc_num].pcheck   = chk_func;
    puart->p[ptc_num].pcatch   = cat_func;

    if ((cat_func == NULL) || (chk_func == NULL) ||
        (puart->p[ptc_num].var.buf == NULL))
        return 0;

    memset (puart->p[ptc_num].var.buf, 0x00, sizeof(unsigned char) * ptc_size);
    return 1;
}

//------------------------------------------------------------------------------
int ptc_grp_init (uart_t *puart, unsigned char ptc_count)
{
    puart->pcnt = ptc_count;

    puart->p = (ptc_func_t *)(malloc(sizeof(ptc_func_t) * ptc_count));

    if (puart->p != NULL) {
        memset (puart->p, 0x00, sizeof(ptc_func_t) * ptc_count);
        return 1;
    }
    return 0;
}

//------------------------------------------------------------------------------
void ptc_grp_close (uart_t *puart)
{
    unsigned char ptc_pos;

    /* destroy pthread for tx / rx */
    {
        int ret;
        ret = pthread_detach(puart->rx_thread);
        printf ("rx thread detach = %d\n", ret);
        ret = pthread_detach(puart->tx_thread);
        printf ("tx thread detach = %d\n", ret);
    }

    for (ptc_pos = 0; ptc_pos < puart->pcnt; ptc_pos++)
        free (puart->p[ptc_pos].var.buf);

    free (puart->p);
    free (puart);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int uart_write (uart_t *puart, unsigned char *w_data, int w_size)
{
    int w_cnt = 0, i;

    for (i = 0; i < w_size; i++) {
        if (queue_put (&puart->tx_q, &w_data[i]))    w_cnt++;
    }
    return w_cnt;
}

//------------------------------------------------------------------------------
int uart_read (uart_t *puart, unsigned char *r_data, int r_size)
{
    int r_cnt = 0, i;

    for (i = 0; i < r_size; i++) {
        if (queue_get (&puart->rx_q, &r_data[i]))    r_cnt++;
    }
    return r_cnt;
}

//------------------------------------------------------------------------------
uart_t *uart_init (const char *dev_name, int baud)
{
    int fd;
    unsigned char buf;
    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    uart_t *puart;

    if ((fd = open(dev_name, O_RDWR)) < 0) {
        printf("%s open error!\n", dev_name);
        return NULL;
    }

    // Read in existing settings, and handle any error
    if(tcgetattr(fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(fd);
        return NULL;
    }

    tty.c_cflag &= ~PARENB;     // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;     // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;      // Clear all bits that set the data size
    tty.c_cflag |= CS8;         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    //tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 seconds), returning as soon as any data is received.
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, baudrate (baud));
    cfsetospeed(&tty, baudrate (baud));

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(fd);
        return NULL;
    }
    while(read(fd, &buf, 1));    // read all if there is data in the serial rx buffer

    /* UART control struct init */
    if ((puart = (uart_t *)(malloc(sizeof(uart_t)))) != NULL) {
        memset (puart, 0x00, sizeof(uart_t));
        puart->fd         = fd;

        puart->tx_q.sp    = 0;
        puart->tx_q.ep    = 0;
        puart->tx_q.size  = DEFAULT_QUEUE_SIZE;
        puart->tx_q.buf   = (unsigned char *)(malloc(DEFAULT_QUEUE_SIZE));

        puart->rx_q.sp    = 0;
        puart->rx_q.ep    = 0;
        puart->rx_q.size  = DEFAULT_QUEUE_SIZE;
        puart->rx_q.buf   = (unsigned char *)(malloc(DEFAULT_QUEUE_SIZE));

        if ((puart->tx_q.buf == NULL) || (puart->rx_q.buf == NULL)) {
            printf ("error : rx/tx queue create error!\n");
            free (puart);
            return NULL;
        }
        /* Create pthread for tx / rx */
        pthread_create(&puart->rx_thread, NULL, rx_thread_func, puart);
        pthread_create(&puart->tx_thread, NULL, tx_thread_func, puart);
        return puart;
    }
    return NULL;
}

//------------------------------------------------------------------------------
void uart_close (uart_t *puart)
{
    if (puart->fd)
        close(puart->fd);

    ptc_grp_close (puart);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
