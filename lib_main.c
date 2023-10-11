//------------------------------------------------------------------------------
/**
 * @file lib_main.c
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
//------------------------------------------------------------------------------
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <getopt.h>

#include "lib_uart.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#if defined(__LIB_UART_APP__)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void print_usage(const char *prog)
{
    puts("");
    printf("Usage: %s [-D:device] [-b:baud] [-t:tx msg] [-r:rx count]\n", prog);
    puts("\n"
         "  -D --Device         Control Device node\n"
         "  -b --baud           UART baudrate\n"
         "  -t --tx control     request tx message.\n"
         "  -r --rx control     request rx get bytes.\n"
         "\n"
         "  e.g) send 'abcd' to tx and wait until get 3 bytes from rx\n"
         "       lib_uart -D /dev/ttyUSB0 -b 115200 -t abcd -r 3\n"
    );
    exit(1);
}

//------------------------------------------------------------------------------
/* Control server variable */
//------------------------------------------------------------------------------
static int   OPT_BAUDRATE       = 115200;
static int   OPT_RX_COUNT       = 0;
static char *OPT_DEVICE_NODE    = NULL;
static char *OPT_TX_MESSAGE     = NULL;

//------------------------------------------------------------------------------
// 문자열 변경 함수. 입력 포인터는 반드시 메모리가 할당되어진 변수여야 함.
//------------------------------------------------------------------------------
static void tolowerstr (char *p)
{
    int i, c = strlen(p);

    for (i = 0; i < c; i++, p++)
        *p = tolower(*p);
}

//------------------------------------------------------------------------------
static void toupperstr (char *p)
{
    int i, c = strlen(p);

    for (i = 0; i < c; i++, p++)
        *p = toupper(*p);
}

//------------------------------------------------------------------------------
static void parse_opts (int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "Device",     0, 0, 'D' },
            { "Baudrate",   0, 0, 'b' },
            { "rx control", 1, 0, 'r' },
            { "tx control", 1, 0, 't' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:b:r:t:h", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
        case 'D':
            OPT_DEVICE_NODE = optarg;
            break;
        case 'b':
            OPT_BAUDRATE = atoi(optarg);
            break;
        case 't':
            OPT_TX_MESSAGE = optarg;
            break;
        case 'r':
            OPT_RX_COUNT = atoi(optarg);
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            break;
        }
    }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    uart_t *puart;
    parse_opts(argc, argv);

    if (OPT_DEVICE_NODE == NULL) {
        printf ("error : control device node is null\n");
        printf ("Use the '-h' command to display help\n");
        return 0;
    }

    if ((puart = uart_init(OPT_DEVICE_NODE, OPT_BAUDRATE)) != NULL) {
        if (OPT_TX_MESSAGE != NULL) {
            int cnt = strlen (OPT_TX_MESSAGE);
            printf ("TX Msg (size = %d) : %s\n", cnt, OPT_TX_MESSAGE);
            if ((cnt = uart_write (puart, (unsigned char *)OPT_TX_MESSAGE, cnt)) != (int)strlen(OPT_TX_MESSAGE))
                printf ("write size error. \n");
            else {
                printf("wait for uart tx (%d usec)...\n", cnt * 200);
                usleep (cnt * 200);
            }
            fflush(stdout);
        }
        if (OPT_RX_COUNT) {
            int rx_cnt = 0;
            char rx_data;
            printf ("Wait RX Msg(size = %d) : ", OPT_RX_COUNT);
            fflush(stdout);

            while (rx_cnt < OPT_RX_COUNT) {
                if (uart_read (puart, (unsigned char *)&rx_data, 1)) {
                    printf ("%c", rx_data);
                    fflush(stdout);
                    rx_cnt++;
                }
                usleep(200);
            }
            puts("");
        }
    }
    uart_close (puart);
    return 0;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#endif  // #if defined(__LIB_UART_APP__)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
