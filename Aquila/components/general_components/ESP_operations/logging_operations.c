#include <stdarg.h>
#include <stdio.h>
#include "wt_alldef.h"
#ifdef TELNET_CONF_MODE
    #include <lwip/sockets.h>
#endif

//Вспомогательная функция для вывода
void print_service_message(int16_t fd, const char *format, ...) {
    char buffer[500];
    va_list args;
    va_start(args, format);
    uint8_t len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

//Печать в терминал
    printf("%s", buffer);
//печать в telnet
#ifdef TELNET_CONF_MODE
    if (fd >= 0) {
        send(fd, buffer, len, 0);
    }
#endif
}