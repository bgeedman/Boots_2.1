#ifndef KINEMATICS_H
#define KINEMATICS_H

#define KINEMATICS_VERSION "0.0.1";

#include "leg.h"

int solve_kinematics_geometric(Leg *leg);
int solve_kinematics_ccd(Leg *leg);
int solve_kinematics_pseudoinverse_jacobian(Leg *leg);

#endif
