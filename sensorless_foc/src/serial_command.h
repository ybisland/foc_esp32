#pragma once

#include "FOC.h"

void serial_command_init(FOC_Controller *foc);
void receive_serial_commands();