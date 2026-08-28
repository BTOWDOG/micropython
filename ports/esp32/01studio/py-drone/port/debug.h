#ifndef __DEBUG__H
#define __DEBUG__H


#define DBG_EVERY(cnt, n, fmt, ...) \
    do { \
        static uint32_t cnt = 0; \
        if (++cnt >= n) { \
            cnt = 0 ; \
            printf(fmt "\r\n", ##__VA_ARGS__); \
        } \
    } while (0)


#endif