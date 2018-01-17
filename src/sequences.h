#ifndef SEQUENCES_H
#define SEQUENCES_H

#define SEQUENCES_H_VERSION "0.0.1"


typedef struct {
    int frame;
    // array of points?
} sequence;

/*
 * static uint32_t seq[][3] = {
 *  {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3},
 * }
 *
 */


void seq_unknown_to_park(void);
void seq_park_to_stand(void);
void seq_stand_to_park(void);
void seq_stand_to_stretch(void);
void seq_stand_to_turn_left(void);
void seq_stand_to_turn_right(void);
void seq_stand_to_walk(void);

void seq_stop_and_center(void);

#endif
