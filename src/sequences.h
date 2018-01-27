#ifndef SEQUENCES_H
#define SEQUENCES_H
#include "commands.pb-c.h"

#define SEQUENCES_H_VERSION "0.0.1"


typedef struct {
    int x;
    int y;
    int z;
} point_t;


point_t seq_unknown_to_park(int, int);//, Command *);
point_t seq_park_to_stand(int, int);//, Command *);
point_t seq_stand_to_park(int, int);//, Command *);
point_t seq_stand_to_stretch(int, int);//, Command *);
point_t seq_stand_to_turn_left(int, int);//, Command *);
point_t seq_stand_to_turn_right(int, int);//, Command *);
point_t seq_stand_to_walk(int, int);//, Command *);

point_t seq_stop_and_center(int, int);//, Command *);

#endif
