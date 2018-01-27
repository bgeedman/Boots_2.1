#include <gsl/gsl_matrix.h>
#include <pthread.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "tools.h"
#include "logger.h"


pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static int fd;

void tools_matrix_print(char *label, gsl_matrix *matrix) {
    if (matrix == NULL) {
        log_info("Null matrix");
        return;
    }
    char buf[1024];
    int i, j;
    int offset = 0;
    offset += snprintf(buf, sizeof(buf), "%s\n", label);
    for (i = 0; i < (int)matrix->size1; i++) {
        for (j = 0; j < (int)matrix->size2; j++) {
            offset += snprintf(buf + offset, sizeof(buf) - offset, "%6.3g ",
                              gsl_matrix_get(matrix, i, j));
        }
        offset += snprintf(buf+offset, sizeof(buf) - offset, "\n");
    }
    log_info("%s", buf);
}



void lock_logger(void *udata, int lock) {
    if (lock) {
        pthread_mutex_lock(&log_mutex);
    } else {
        pthread_mutex_unlock(&log_mutex);
    }
}


int open_serial_port(char *port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    struct termios tty;
    if (fd < 0) {
        log_warn("Failed to open serial port");
        return -1;
    }

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty)) {
        log_warn("Failed to get attributs");
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
        log_warn("Failed to set attributes");
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
