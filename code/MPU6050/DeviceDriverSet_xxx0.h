#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_

#include <arduino.h>

/*Motor*/
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, 
                                     boolean direction_B, uint8_t speed_B, 
                                     boolean controlED                     
  );                                                                       
private:

#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 7
#define PIN_Motor_AIN_1 8

public:
#define speed_Max 255 // maximum speed of motor rotation

// defining flag variables
// (motor rotation direction)
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};

#endif
