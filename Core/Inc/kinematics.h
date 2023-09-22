#pragma once
#include "main.h"
#include <math.h>
void reverse_kinematic(Kinematic_parameters_t kinematic);

void direct_kinematic(Kinematic_parameters_t kinematic);

void set_kinematic_target(Kinematic_parameters_t kinematic, float Vx, float Vy, float angular);

void matrixMultiplyM2M(float *m1, char rows1, char columns1, float *m2, char rows2, char columns2, float *new_m);
