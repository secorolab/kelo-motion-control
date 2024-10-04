#include "kelo_motion_control/PowerManagementUnit.h"

uint32_t int_to_command(int input) {
  switch (input) {
    case 1:
      return BASE_ON;
    case 2:
      return TORSO_ON;
    case 3:
      return BASE_AND_TORSO_ON;
    case 4:
      return BASE_AND_TORSO_OFF;
    case 5:
      return BASE_DON_TORSO_ON;
    default:
      return 0;  // Unknown command
  }
}

void read_pmu_data(EthercatConfig *ethercat_config, double *current,
                   double *voltage, double *power, uint16_t *status) {
  PowerManagementUnitProcessDataInput *ecData =
      (PowerManagementUnitProcessDataInput *)ethercat_config->ecx_slave[0]
          .inputs;
  *current = ecData->CURRENT;
  *voltage = ecData->VOLTAGE;
  *power = ecData->POWER;
  *status = ecData->STATUS;
}

void write_pmu_data(EthercatConfig *ethercat_config, uint16_t shutdown,
                    uint32_t command) {

  PowerManagementUnitProcessDataOutput msg;
  memset(&msg, 0, sizeof(msg));
  msg.SHUTDOWN = shutdown;
  msg.COMMAND = command;

  PowerManagementUnitProcessDataOutput *ecData =
      (PowerManagementUnitProcessDataOutput *)ethercat_config->ecx_slave[0]
          .outputs;
  memcpy(ecData, &msg, sizeof(msg));
}

void shutdown_pmu(EthercatConfig *ethercat_config, uint8_t delay) {
  write_pmu_data(ethercat_config, 0x80 | delay, 0);
}

void command_pmu(EthercatConfig *ethercat_config, uint32_t command) {
  write_pmu_data(ethercat_config, 0, command);
}