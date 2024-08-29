#ifndef FMS_INTERFACE_H
#define FMS_INTERFACE_H

#include "rc_control.h"
#include "gcs_control.h"
#include "main.h"
#include "ins_interface.h"

typedef struct
{
  const RC_ctrl_t *fms_cmd_bus;
  const INS_Bus *fms_ins_bus;
//   const CTRL_Bus *fms_ctrl_bus;
} FMS_Bus;

void fms_interface_init();
void fms_interface_step(uint32_t timestamp);


#endif