#include "kelo_motion_control/KeloMotionControl.h"

void init_torque_control_state(TorqueControlState *state, int N, int M)
{
  state->A = gsl_matrix_calloc(N, M);
  state->A_inv_T = gsl_matrix_calloc(M, N);
  state->A_tmp = gsl_matrix_calloc(N, M);
  state->A_inv_T_tmp = gsl_matrix_calloc(M, N);
  state->work = gsl_vector_calloc(N);
  state->W = gsl_matrix_calloc(N, N);
  state->K = gsl_matrix_calloc(M, M);
  state->u = gsl_vector_calloc(N);
  state->V = gsl_matrix_calloc(N, N);
  state->u_inv = gsl_matrix_calloc(N, N);
  state->b = gsl_matrix_calloc(N, 1);
  state->b_verify = gsl_matrix_calloc(N, 1);
}

void set_weight_matrix(TorqueControlState *state, int N, int M)
{
  for (size_t i = 0; i < M; i++)
  {
    gsl_matrix_set(state->K, i, i, 1.0);

    if (i < N)
    {
      gsl_matrix_set(state->W, i, i, 1.0);
    }
  }
}

void set_platform_force(TorqueControlState *state, double *platform_force, int N)
{
  for (size_t i = 0; i < N; i++)
  {
    gsl_matrix_set(state->b, i, 0, platform_force[i]);
  }
}

void compute_wheel_torques(int nWheels, double wheel_radius, double castor_offset, double half_wheel_distance,
                           double *wheel_coordinates, TorqueControlState *state,
                           double *pivot_angles, double *wheel_torques, int N, int M)
{
  if (wheel_coordinates == NULL || state == NULL || pivot_angles == NULL || wheel_torques == NULL)
  {
    printf("[KeloMotionControl] Error: Invalid input arguments.\n");
    exit(1);
  }
  platform_force_to_wheel_torques(nWheels, wheel_radius, castor_offset, half_wheel_distance,
                                  wheel_coordinates, wheel_torques, pivot_angles, state->b,
                                  state->b_verify, state->A, state->A_inv_T, state->A_tmp,
                                  state->A_inv_T_tmp, state->work, state->W, state->K, state->u,
                                  state->V, state->u_inv, M, N, false);
}

void free_torque_control_state(TorqueControlState *state)
{
  gsl_matrix_free(state->A);
  gsl_matrix_free(state->A_inv_T);
  gsl_matrix_free(state->A_tmp);
  gsl_matrix_free(state->A_inv_T_tmp);
  gsl_vector_free(state->work);
  gsl_matrix_free(state->W);
  gsl_matrix_free(state->K);
  gsl_vector_free(state->u);
  gsl_matrix_free(state->V);
  gsl_matrix_free(state->u_inv);
  gsl_matrix_free(state->b);
  gsl_matrix_free(state->b_verify);
}