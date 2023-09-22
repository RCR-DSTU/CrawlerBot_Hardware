#include "kinematics.h"

#define R 0.0877
#define LEN	0.263

static float matrix_kinematic[2][3] =
   { {R, 0, LEN},
	 {-R, 0, -LEN} };


void direct_kinematic(Kinematic_parameters_t kinematic)
{

}


// Target Linear&Angular Velocity to --> engines speed
void reverse_kinematic(Kinematic_parameters_t kinematic)
{
	float tmp_m_rotating[3][3] =
	{ {cosf(kinematic.current_moving[2]), -sinf(kinematic.current_moving[2]), 0},
	  {sinf(kinematic.current_moving[2]), cosf(kinematic.current_moving[2]),  0},
	  {0, 								 0, 						        1} };

	float localVelocity[3], outputVelocity[3];

	matrixMultiplyM2M(&tmp_m_rotating[0][0], 3, 3, &kinematic.target_moving[0], 3, 1, &localVelocity[0]);
	if(!kinematic.kinematic_on)
	{
		matrixMultiplyM2M(&matrix_kinematic[0][0], 2, 3, &localVelocity[0], 3, 1, &outputVelocity[0]);
	} else 	matrixMultiplyM2M(&matrix_kinematic[0][0], 2, 3, &localVelocity[0], 3, 1, &kinematic.kinematic_out[0]);
}

void set_kinematic_target(Kinematic_parameters_t kinematic, float Vx, float Vy, float angular)
{
	kinematic.target_moving[0] = Vx;
	kinematic.target_moving[1] = Vy;
	kinematic.target_moving[2] = angular;
}

//Операция перемножения матриц
void matrixMultiplyM2M(float *m1, char rows1, char columns1, float *m2, char rows2, char columns2, float *new_m)
{
float Sum;
char i,j,k;

  if (columns1 != rows2)
      *new_m = 0;
  else
    {
      for(i = 0; i < rows1; i++)
        for (j = 0; j < columns2; j++)
        {
            Sum = 0;
            for(k = 0; k < columns1; k++)
                Sum+= (*(m1+columns1*i+k)) * (*(m2+columns2*k+j));
            *(new_m+columns2*i+j) = Sum;
        }
    }
}
