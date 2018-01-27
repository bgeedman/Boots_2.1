#ifndef SERVER_H
#define SERVER_H

#define SERVER_H_VERSION "0.0.1"

#include <pthread.h>
#include "commands.pb-c.h"

extern pthread_mutex_t cmd_mutex;
extern Command *command;

typedef struct serv_args_t {
    char *address;
    short port;
//    Command **cmd;
}serv_args_t;


int create_server_thread(const char *address, short port);//, Command **cmd);
void *server_thread(void *args);

#endif
