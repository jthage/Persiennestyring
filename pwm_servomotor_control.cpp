#include <servomotor_contol.h>

void Configure_GPIO_PINs(){
	// enable clock to port D
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	// Make DIR_PINs to GPIO
	PORTD->PCR[DIR_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[DIR_PIN] |= PORT_PCR_MUX(3);

	//set GPIOs to outputs
	PTA->PDDR |=  MASK(DIR_PIN);
}

/* Function to determine direction
To turn the motor clockwise input a 1 (set it HIGH).
To turn counterclockwise input 0 (set it LOW).
*/

void Control_Motor_Dir(uint8_t pin_on){
	if(pin_on)
		PTA->PSOR = MASK(DIR_PIN);
	else
		PTA->PCOR = MASK(DIR_PIN);
}

void Configure_TPM_PIN(uint16_t period){
	// enable clock to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Set STEP_PIN to FTM/TPM/PWM
	PORTD->PCR[STEP_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[STEP_PIN] |= PORT_PCR_MUX(4);

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

void Increase_Motor_Speed(uint16_t motor_speed){
	uint16_t i = 0;
	for (i=0; i<motor_speed; i++){
				TPM0->CONTROLS[1].CnV = i;
			}
}

void Decrease_Motor_Speed(uint16_t motor_speed){
	uint16_t i = 0;
	for (i=0; i<motor_speed-1; i--){
				TPM0->CONTROLS[1].CnV = i;
			}
}
