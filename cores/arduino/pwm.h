#ifndef _HAL_PWM_H_
#define _HAL_PWM_H_

#define SBIT_CNTEN     0
#define SBIT_PWMEN     2
#define SBIT_PWMMR0R   1

#define PWM_1          0  //P2_00 (0-1 Bits of PINSEL4)
#define PWM_2          2  //P2_01 (2-3 Bits of PINSEL4)
#define PWM_3          4  //P2_02 (4-5 Bits of PINSEL4)
#define PWM_4          6  //P2_03 (6-7 Bits of PINSEL4)
#define PWM_5          8  //P2_04 (8-9 Bits of PINSEL4)
#define PWM_6          10 //P2_05 (10-11 Bits of PINSEL4)

void HAL_pwm_init(void);

#endif // _HAL_PWM_H_
