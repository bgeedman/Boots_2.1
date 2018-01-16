#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <pthread.h>

#include "server.h"
#include "commands.pb-c.h"
#include "logger.h"

pthread_mutex_t cmd_mutex = PTHREAD_MUTEX_INITIALIZER;



static int read_n_bytes(int fd, int len, void *buf) {
    int nread = 0;
    int offset = 0;
    while ((nread = read(fd, buf + offset, len - offset))) {
        offset += nread;
    }
    return offset;
}





void *server_thread(void *args) {
    int serv_fd;
    int conn_fd;

    serv_args_t *arg = (serv_args_t *)args;

    struct sockaddr_in serv_addr, cli_addr;
    socklen_t slen;

    int32_t len;
    uint8_t buf[1024];
    int bytes_read;

    log_debug("Thread - creating new socket");
    if ((serv_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        log_fatal("Thread - failed to create new socket");
        return NULL;
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    memset(&cli_addr, 0, sizeof(cli_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(arg->address);
    serv_addr.sin_port = ntohs(arg->port);

    log_debug("Thread - binding socket");
    if (bind(serv_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) {
        log_fatal("Thread - failed to bind socket");
        return NULL;
    }
    log_debug("Thread - listening for new connection");
    listen(serv_fd, 1); // should error trap this
    conn_fd = accept(serv_fd, (struct sockaddr*)&cli_addr, &slen);
    if (conn_fd < 0) {
        log_fatal("Thread - failed to accept");
        return NULL;
    }
    while (read(conn_fd, &len, sizeof(int32_t))) {
        len = ntohl(len);
        if (len >= 1024) {
            log_fatal("Thread - too much data detected: %d", len);
            close(conn_fd);
            return NULL;
        }
        if ((bytes_read = read_n_bytes(conn_fd, len, buf)) != len) {
            log_fatal("Thread - read incorrect number of bytes");
            log_fatal("Thread - Needed %d. Read: %d", len, bytes_read);
            close(conn_fd);
            return NULL;
        }
        pthread_mutex_lock(&cmd_mutex);
        *(arg->cmd) = command__unpack(NULL, bytes_read, buf);
        pthread_mutex_unlock(&cmd_mutex);
    }
    close(conn_fd);
    return NULL;
}




