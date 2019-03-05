/*
 * servomotor_contol.h
 *
 *  Created on: 21. jan. 2019
 *      Author: JMT
 */

#include <stdio.h>
#include <MKL25Z4.h>
#include <stdint.h>

#define STEP_PIN (4)
#define DIR_PIN (5)

/* motor max speed used in
 * void Increase_Motor_Speed(uint16_t motor_speed);
 * and
 * void Decrease_Motor_Speed(uint16_t motor_speed);
 */
#define SPEED (48000)

#define MASK(x) (1UL << (x))


#ifndef SERVOMOTOR_CONTOL_H_
#define SERVOMOTOR_CONTOL_H_

// set up GPIO pin(s)
void Configure_GPIO_PINs();
// set up FTM/TPM/PWM pin(s)
void Configure_TPM_PIN(uint16_t period);
// rotational direction of the motor - 1 = clockwise 0 counterclockwise
void Control_Motor_Dir(uint8_t pin_on);
/* set the max speed of the motor limit is 48000.
 * the motor accelerates until max speed is reached
 */
void Increase_Motor_Speed(uint16_t motor_speed);
/*
 * the motor deaccelerates from max speed until it does not move
 */
void Decrease_Motor_Speed(uint16_t motor_speed);

#endif /* SERVOMOTOR_CONTOL_H_ */
