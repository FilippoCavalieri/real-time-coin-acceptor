#include "pico/stdlib.h"
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <time.h>
#include "pico/stdlib.h"

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t t = time_us_64();
    tp->tv_sec  = t / 1000000ULL;
    tp->tv_nsec = (t % 1000000ULL) * 1000ULL;
    return 0;
}

void usleep(uint64_t us)
{
    sleep_us(us);
}


bool pico_usb_transport_open(struct uxrCustomTransport * transport)
{
    static bool initialized = false;
    if (!initialized)
    {
        stdio_init_all();   // init USB CDC
        initialized = true;
    }
    return true;
}

bool pico_usb_transport_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t pico_usb_transport_write(struct uxrCustomTransport * transport,
                                uint8_t * buf, size_t len, uint8_t * errcode)
{
    size_t written = 0;
    for (size_t i = 0; i < len; i++)
    {
        int c = putchar_raw(buf[i]);  // raw avoids CR/LF translation
        if (c < 0)
        {
            *errcode = 1;
            break;
        }
        written++;
    }
    return written;
}

size_t pico_usb_transport_read(struct uxrCustomTransport * transport,
                               uint8_t * buf, size_t len, int timeout, uint8_t * errcode)
{
    uint64_t start = time_us_64();
    size_t received = 0;

    while (received < len)
    {
        int c = getchar_timeout_us(0);  // non-blocking
        if (c != PICO_ERROR_TIMEOUT)
        {
            buf[received++] = (uint8_t)c;
        }
        else
        {
            if ((time_us_64() - start) > (uint64_t)timeout * 1000ULL)
            {
                break; // timeout expired
            }
        }
    }

    if (received == 0)
    {
        *errcode = 1;
    }
    return received;
}
