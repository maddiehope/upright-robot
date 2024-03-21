#include "DeviceDriverSet_xxx0.h"

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  // initializing motor pins
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
}


/* Motor_control */
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(
  boolean direction_A, 
  uint8_t speed_A, 
  boolean direction_B, uint8_t speed_B, 
  boolean controlED  
)                                   
{
  if (controlED == control_enable)
  {
    { //A...Right

      switch (direction_A) 
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);

        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        break;
      }
    }
  }
  else
  {
    return;
  }
}
