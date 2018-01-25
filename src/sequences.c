#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>


#include "leg.h"
#include "logger.h"
#include "sequences.h"


point_t seq_unknown_to_park(int cur_frame, int leg) {
    log_trace("setting sequence unknown_to_park");
    static int NUMBER_OF_FRAMES = 0;
    static point_t seq[] = {
        {-200, 40, 40}, {200, 40, 40}, {-200, -40, 40}, {200, -40, 40}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    // return the correct index for the frame. multiply by the number of
    // legs and add the current leg
    return seq[4 * cur_frame + leg];
}




point_t seq_park_to_stand(int cur_frame, int leg) {
    log_trace("setting sequence park_to_stand");
    static int NUMBER_OF_FRAMES = 2;
    static point_t seq[] = {
        {-200, 40, 40}, {200, 40, 40}, {-200, -40, 40}, {200, -40, 40},
        {-200, 200, 0}, {200, 200, 0}, {-200, -200, 0}, {200, -200, 0},
        {-200, 200, 100}, {200, 200, 100}, {-200, -200, 100}, {200, -200, 100},
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}



point_t seq_stand_to_park(int cur_frame, int leg) {
    log_trace("setting sequence stand_to_park");
    static int NUMBER_OF_FRAMES = 2;
    static point_t seq[] = {
        {-200, 200, 100}, {200, 200, 100}, {-200, -200, 100}, {200, -200, 100},
        {-200, 200, 0}, {200, 200, 0}, {-200, -200, 0}, {200, -200, 0},
        {-200, 40, 40}, {200, 40, 40}, {-200, -40, 40}, {200, -40, 40}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}





point_t seq_stand_to_stretch(int cur_frame, int leg ) {
    // read the command
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_turn_left(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_turn_right(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_walk(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stop_and_center(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}
