#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "commands.pb-c.h"
#include "logger.h"
#include "server.h"


pthread_mutex_t cmd_mutex = PTHREAD_MUTEX_INITIALIZER;
Command *command = NULL;


static int read_n_bytes(int fd, int len, void *buf) {
    int nread = 0;
    int offset = 0;
    while ((nread = read(fd, buf + offset, len - offset))) {
        offset += nread;
    }
    return offset;
}



int create_server_thread(const char *address, short port) {
    struct serv_args_t *serv_a = NULL;
    int str_len;
    int error;
    pthread_t tid;

    if ((serv_a = malloc(sizeof(serv_args_t))) == NULL) {
        error = errno;
        log_fatal("Failed to malloc server args: %s", strerror(errno));
        errno = error;
        return -1;
    }

    str_len = strlen(address) + 1;

    if ((serv_a->address = malloc(sizeof(str_len))) == NULL) {
        error = errno;
        log_fatal("Failed to malloc address: %s", strerror(errno));
        errno = error;
        goto FAIL;
    }

    serv_a->port = port;
    strncpy(serv_a->address, address, str_len);

    if (pthread_create(&tid, NULL, server_thread, serv_a)) {
        error = errno;
        log_fatal("Failed to create server thread: %s", strerror(errno));
        errno = error;
        goto FAIL;
    }

    if (pthread_detach(tid)) {
        error = errno;
        log_fatal("Failed to detach server thread: %s", strerror(errno));
        errno = error;
        goto FAIL;
    }
    return 0;

FAIL:
    free(serv_a->address);
    free(serv_a);
    return -1;
}



void *server_thread(void *args) {
    int serv_fd;
    int conn_fd;
    int32_t len;
    uint8_t buf[1024];
    int bytes_read;
    int error;

    serv_args_t *arg = (serv_args_t *)args;

    struct sockaddr_in serv_addr, cli_addr;
    socklen_t slen;

    if ((serv_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        error = errno;
        log_fatal("Failed to create new socket: %s", strerror(errno));
        errno = error;
        return NULL;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    memset(&cli_addr, 0, sizeof(cli_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(arg->address);
    serv_addr.sin_port = ntohs(arg->port);

    if (bind(serv_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) {
        error = errno;
        log_fatal("Failed to bind socket: %s", strerror(errno));
        errno = error;
        return NULL;
    }

    if (listen(serv_fd, 1) < 0) {
        error = errno;
        log_fatal("Failed to listen: %s", strerror(errno));
        errno = error;
        return NULL;
    }

    while (1) {
        conn_fd = accept(serv_fd, (struct sockaddr*)&cli_addr, &slen);

        if (conn_fd < 0) {
            error = errno;
            log_fatal("Failed to accept new connection: %s", strerror(errno));
            errno = error;
            return NULL;
        }

        while (read(conn_fd, &len, sizeof(int32_t)) > 0) {
            len = ntohl(len);
            if (len >= 1024) {
                log_fatal("Too much data detected: %d. Closing server!", len);
                // this should send a signal to kill everything
                // pthread_exit(NULL);
                close(conn_fd);
                return NULL;
            }
            if ((bytes_read = read_n_bytes(conn_fd, len, buf)) != len) {
                log_fatal("Read incorrect number of bytes");
                log_fatal("Needed %d. Read: %d", len, bytes_read);
                // this also should send a signal to kill everything
                // pthread_exit(NULL);
                close(conn_fd);
                return NULL;
            }
            pthread_mutex_lock(&cmd_mutex);
            command = command__unpack(NULL, bytes_read, buf);
            pthread_mutex_unlock(&cmd_mutex);
        }
        // Check on errno

        close(conn_fd);
    }
    return NULL;
}
