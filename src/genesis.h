#ifndef GENESIS_H
#define GENESIS_H

#define GENESIS_VERSION "0.0.1"

#include "leg.h"

int genesis_leg_creation(Leg **front_left, Leg **front_right, Leg **back_left, Leg **back_right);
void genesis_leg_destruction(Leg *front_left, Leg *front_right, Leg *back_left, Leg *back_right);

#endif
