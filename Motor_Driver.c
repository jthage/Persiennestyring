/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    PWM_BLUE-.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#define PWM_PERIOD (48000)
#define MORTOR_POS (1)

void Init_Motor_PWM(uint16_t period) {
	//enable clock to PORTD, TM0
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;;
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// set pin to FTM
	// Blue FTMO_CH1, MUX Alt 4

	PORTD->PCR[MORTOR_POS] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[MORTOR_POS] |= PORT_PCR_MUX(4);

	// configure TPM
	// set clock source for tpm: 48 MHz

	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	// load the counter and mod
	TPM0->MOD = period-1;
	// Set TPM count direction up with a divide by 2 prescaler
	TPM0->SC = TPM_SC_PS(1);
	// continue operation i debug mode
	TPM0->CONF |= TPM_CONF_DBGMODE(3);
	// set channel 1 to edge-aligned low-true PWM
	TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	// set initial duty cycle
	TPM0->CONTROLS[1].CnV = 0;
	// start TPM
	TPM0->SC |= TPM_SC_CMOD(1);
}

int main (void){
	uint16_t i=0;
	volatile int32_t delay;
	Init_Motor_PWM(PWM_PERIOD);
	// LED flash forever
	while(1){
		for (i=0; i<PWM_PERIOD; i++){
			TPM0->CONTROLS[1].CnV = i;
			for (delay=0; delay<100; delay++)
				;
		}
		// Dim LED
		for (i=PWM_PERIOD-1; i>0; i--){
			TPM0->CONTROLS[1].CnV = i;
			for (delay=0; delay<100; delay++)
				;
		}
	}
}
