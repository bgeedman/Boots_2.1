#include <errno.h>
#include <gsl/gsl_matrix.h>
#include <pthread.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "tools.h"
#include "logger.h"
#include "sequences.h"


pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static int fd;


void lock_logger(void *udata, int lock) {
    if (lock) {
        pthread_mutex_lock(&log_mutex);
    } else {
        pthread_mutex_unlock(&log_mutex);
    }
}


int open_serial_port(char *port) {
    struct termios tty;
    int error;

    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        error = errno;
        log_fatal("Failed to open serial port: %s", strerror(errno));
        errno = error;
        return -1;
    }

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty)) {
        error = errno;
        log_fatal("Failed to get attributes: %s", strerror(errno));
        errno = error;
        return -1;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;

    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if (tcsetattr(fd, TCSANOW, &tty)) {
        error = errno;
        log_fatal("Failed to set attributes: %s", strerror(errno));
        errno = error;
        return -1;
    }
    return 0;
}


int write_command(char *buf) {
    int len = strlen(buf);
    int offset = 0;
    if (len <= 0) {
        log_warn("Buffer length is 0");
        return -1;
    }
    while ((offset = write(fd, buf + offset, len - offset)));
    return 0;
}
