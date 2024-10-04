#ifndef KELO_PMU_H
#define KELO_PMU_H

#include "kelo_motion_control/EthercatCommunication.h"

// define command bits
// command: bits 32-8 reserved
// command: bits 7 - DO8 - Default: 0 - not connected
// command: bits 6 - DO7 - Default: 0 - not connected
// command: bits 5 - DO6 - Default: 0 - torso power
// command: bits 4 - DO5 - Default: 0 - not connected
// command: bits 3 - DO4 - Default: 0 - not connected
// command: bits 2 - DO3 - Default: 1 - platform power
// command: bits 1 - DO2 - Not controllable
// command: bits 0 - DO1 - Not controllable

#define BASE_ON 0b00000111
#define TORSO_ON 0b00100100
#define BASE_DON_TORSO_ON 0b00100000  // base (default on at startup), torso on
#define BASE_AND_TORSO_ON 0b00100111
#define BASE_AND_TORSO_OFF 0b00000100

// command to string
#define COMMAND_TO_STRING(command)                          \
  ((command == BASE_ON)              ? "BASE_ON"            \
   : (command == TORSO_ON)           ? "TORSO_ON"           \
   : (command == BASE_AND_TORSO_ON)  ? "BASE_AND_TORSO_ON"  \
   : (command == BASE_AND_TORSO_OFF) ? "BASE_AND_TORSO_OFF" \
   : (command == BASE_DON_TORSO_ON)  ? "BASE_DON_TORSO_ON"  \
                                     : "UNKNOWN")

typedef struct PACKED {
  uint16_t STATUS;      // Status bits
  uint64_t TIME_STAMP;  // EtherCAT timestamp ms
  float CURRENT;        // Total current consumption
  float VOLTAGE;        // System Voltage
  float POWER;          // Total power consumption of the system
  float PARAM1;         // Generic data, might be used for different purposes
  uint32_t PARAM2;      // Generic data, might be used for different purposes
} PowerManagementUnitProcessDataInput;

typedef struct PACKED {
  uint16_t
      SHUTDOWN;  // bit 15-8 reserved, bit 7 shutdown, bit 6-0 delay in seconds
  uint32_t COMMAND;
} PowerManagementUnitProcessDataOutput;

uint32_t int_to_command(int input);

void read_pmu_data(EthercatConfig *ethercat_config, double *current,
                   double *voltage, double *power, uint16_t *status);

void write_pmu_data(EthercatConfig *ethercat_config, uint16_t shutdown,
                    uint32_t command);

void shutdown_pmu(EthercatConfig *ethercat_config, uint8_t delay);

void command_pmu(EthercatConfig *ethercat_config, uint32_t command);

#endif  // KELO_PMU_H