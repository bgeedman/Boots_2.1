#ifndef SEQUENCES_H
#define SEQUENCES_H

#define SEQUENCES_H_VERSION "0.0.1"

#include "leg.h"

extern unsigned long frame;

typedef struct {
    int x;
    int y;
    int z;
} point_t;


point_t seq_unknown_to_park(int, Leg *);

point_t seq_park_to_stand(int, Leg *);
point_t seq_stand_to_park(int, Leg *);


point_t seq_stand_to_stretch(int, Leg *);


/*  Turn function declaration */
point_t seq_stand_to_turn_left_trot(int, Leg *);
point_t seq_stand_to_turn_right_trot(int, Leg *);
point_t seq_stand_to_turn_left_crawl(int, Leg *);
point_t seq_stand_to_turn_right_crawl(int, Leg *);
point_t seq_stand_to_turn_left_creep(int, Leg *);
point_t seq_stand_to_turn_right_creep(int, Leg *);


/*  Walk function declaration */
point_t seq_stand_to_walk_trot(int, Leg *);
point_t seq_stand_to_walk_crawl(int, Leg *);
point_t seq_stand_to_walk_creep(int, Leg *);



point_t seq_stop_and_center(int, Leg *);

#endif
