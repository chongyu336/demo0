#include "fms_interface.h"


FMS_Bus fms_bus;
extern RC_ctrl_t rc_ctrl;
extern GCS_ctrl_t gcs_ctrl;
extern INS_Bus ins_bus;
extern bool rc_updated;
extern bool gcs_updated;

void fms_interface_init()
{
  //初始化遥控器和上位机控制
  rc_control_init();
  gcs_control_init();
}



void fms_interface_step(uint32_t timestamp)
{
  //遥控器的优先级高
  if (rc_updated)
  {
    fms_bus.fms_cmd_bus = &rc_ctrl;
  }
  else
  {
    ms_bus.fms_cmd_bus = &gcs_ctrl;
  }
  fms_bus.fms_cmd_bus->timestamp = timestamp;
  fms_bus.fms_ins_bus = &ins_bus;
  // fms_bus.fms_ctrl_bus = &ctrl_bus;

}
