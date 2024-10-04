#include "kelo_motion_control/mediator.h"
#include <stdlib.h>

int main(int argc, char *argv[])
{
  // read input to command pmu
  // inputs 1-4
  // 1 - BASE_ON
  // 2 - TORSO_ON
  // 3 - BASE_AND_TORSO_ON
  // 4 - BASE_AND_TORSO_OFF
  // 5 - BASE_DON_TORSO_ON
  int command = 0;
  if (argc > 1)
  {
    command = atoi(argv[1]);
  }
  else
  {
    printf("Usage: %s <command>\n", argv[0]);
    printf("  command: 1 - BASE_ON\n");
    printf("  command: 2 - TORSO_ON\n");
    printf("  command: 3 - BASE_AND_TORSO_ON\n");
    printf("  command: 4 - BASE_AND_TORSO_OFF\n");
    printf("  command: 5 - BASE_DON_TORSO_ON\n");
    return 1;
  }
  
  uint32_t command_val = int_to_command(command);

  printf("Command: %s\n", COMMAND_TO_STRING(command_val));

  KeloBaseConfig *kelo_base_config = calloc(1, sizeof(*kelo_base_config));
  kelo_base_config->nWheels = 4;
  kelo_base_config->index_to_EtherCAT = (int[4]){3, 4, 6, 7};
  kelo_base_config->radius = 0.115 / 2;
  kelo_base_config->castor_offset = 0.01;
  kelo_base_config->half_wheel_distance = 0.0775 / 2;
  kelo_base_config->wheel_coordinates = (double[8]){0.195, 0.21, -0.195, 0.21, -0.195, -0.21, 0.195, -0.21};
  kelo_base_config->pivot_angles_deviation = (double[4]){0.0, 0.0, 0.0, 0.0};
  
  EthercatConfig *ethercat_config = calloc(1, sizeof(*ethercat_config));
  init_ecx_context(ethercat_config);

  char ifname[] = "eno1";
  int result = 0;

  establish_kelo_base_connection(kelo_base_config, ethercat_config, ifname, command_val, &result);

  double current, voltage, power;
  uint16_t status;

  read_pmu_data(ethercat_config, &current, &voltage, &power, &status);
  printf("Current: %f\n", current);
  printf("Voltage: %f\n", voltage);
  printf("Power: %f\n", power);

  // status: bits 15-3 reserved, bit 2 Emergency switch, bit 1 MSB, bit 0 LSB
  printf("Status: \n");
  printf("  Emergency switch: %d\n", (status >> 2) & 1);
  printf("  MSB: %d\n", (status >> 1) & 1);
  printf("  LSB: %d\n", status & 1);

  free(kelo_base_config);
  free(ethercat_config);

  return 0;
}