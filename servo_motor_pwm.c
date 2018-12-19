/**
 * @file    servo_motor_pwm.c
 * @brief   Application entry point.
 * author: Jørgen
 */
#include <stdio.h>
#include "board.h"
#include "clock_config.h"
#include "motor_pwm.h"

//set pins
#define C0_SHIFT (3)

void init_motor_PWM(uint16_t period){
	// enable clock on port C, TPM0
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	//make pins GPIO
	PORTC->PCR[C0_SHIFT] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[C0_SHIFT] |= PORT_PCR_MUX(1);

	//configure TPM/PWM
	//set clock source for tpm: 48MHz
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	//load the counter and mod
	TPM0->MOD = period-1;
	//set TPM count direction to up with divide by prescaler
	TPM0->SC |= TPM_SC_PS(1);
	//continue configure in debug mode
	TPM0->CONF |= TPM_CONF_DBGMODE(3);
	//set channel 1 to edge-aligned low_true PWM
	TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	//set initial duty cycle
	TPM0->CONTROLS[1].CnV = 0;
	//Start TPM
	TPM0->SC |= TPM_SC_CMOD(1);
}

void Delay(volatile unsigned int time_del){
	while(time_del--){
		;
	}
}
