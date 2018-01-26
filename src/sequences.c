#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>


#include "leg.h"
#include "logger.h"
#include "sequences.h"
#include "commands.pb-c.h"
#include "server.h"


point_t seq_unknown_to_park(int cur_frame, int leg, Command *cmd) {
    log_trace("unknown_to_park: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 0;
    static point_t seq[] = {
        {-150, 40, 30}, {150, 40, 30}, {-150, -50, 30}, {150, -40, 30}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}




point_t seq_park_to_stand(int cur_frame, int leg, Command *cmd) {
    log_trace("park_to_stand: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 9;
    static point_t seq[] = {
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
        // raise the body a little bit
        {-150, 40, 70}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        // move the front left leg
        {-150, 150, 0}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        {-150, 150, 70}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        // move the front right leg
        {-150, 150, 70}, {150, 150, 0}, {-150, -40, 70}, {150, -40, 70},
        {-150, 150, 70}, {150, 150, 70}, {-150, -40, 70}, {150, -40, 70},
        // move the back left leg
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 0}, {150, -40, 70},
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -40, 70},
        // move the back right leg
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -150, 0},
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -150, 70}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}



point_t seq_stand_to_park(int cur_frame, int leg, Command *cmd) {
    log_trace("stand_to_park: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 9;
    static point_t seq[] = {
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -150, 70},
        // move the back right leg
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -40, 30},
        {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -40, 70},
        // move the back left leg
        {-150, 150, 70}, {150, 150, 70}, {-150, -40, 30}, {150, -40, 70},
        {-150, 150, 70}, {150, 150, 70}, {-150, -40, 70}, {150, -40, 70},
        // move the front right leg
        {-150, 150, 70}, {150, 40, 30}, {-150, -40, 70}, {150, -40, 70},
        {-150, 150, 70}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        // move the front left leg
        {-150, 40, 30}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        {-150, 40, 70}, {150, 40, 70}, {-150, -40, 70}, {150, -40, 70},
        // lower the body a bit
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}





point_t seq_stand_to_stretch(int cur_frame, int leg, Command *cmd) {
    float roll, pitch, yaw;
    int delta_x, delta_y;

    pthread_mutex_lock(&cmd_mutex);
        roll = cmd->roll;
        pitch = cmd->ptich;
        yaw = cmd->yaw;
        delta_x = cmd->delta_x;
        delta_y = cmd->delta_y;
    pthread_mutex_unlock(&cmd_mutex);
    // in order for us to be here, we KNOW the leg must be in the stand pos
    // {-150, 150, 70}, {150, 150, 70}, {-150, -150, 70}, {150, -150, 70}
    // apply the translation
    // apply the rotation
    // return the new point
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_turn_left(int cur_frame, int leg, Command *cmd) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_turn_right(int cur_frame, int leg, Command *cmd) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_walk(int cur_frame, int leg, Command *cmd) {
    return (point_t){0, 0, 0};
}


point_t seq_stop_and_center(int cur_frame, int leg, Command *cmd) {
    return (point_t){0, 0, 0};
}
