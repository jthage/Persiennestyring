/*
 * SPI.c
 *
 *  Created on: 9. nov. 2018
 *      Author: andreas
 */

#include "SPI.h"



void SPI_Init()
{
	SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK; // Enable clock for SPI1
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //Enable clock for Port E

	//Ensure that the SPI system is off, before the configturesion.
	SPI1->C1 &= ~SPI_C1_SPE_MASK;

	/*
	 * Enable the pins for the SPI communikation
	 */
	//SPI1_SCK
	PORTE->PCR[SPI1_SCK_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[SPI1_SCK_PIN] |= PORT_PCR_MUX(2);

	//SPI1_MOSI
	PORTE->PCR[SPI1_MOSI_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[SPI1_MOSI_PIN] |= PORT_PCR_MUX(5);

	//SPI1_MISO
	PORTE->PCR[SPI1_MISO_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[SPI1_MISO_PIN] |= PORT_PCR_MUX(5);

	//SPI1_PCSS
	PORTE->PCR[SPI1_SS_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[SPI1_SS_PIN] |= PORT_PCR_MUX(1);

	PTE->PDDR |= (1 << SPI1_SS_PIN);

	/*
	 * SPI control register:
	 * MSTR: Enable the device as a master
	 * SSOE: Works with MODFEN in C2 and MSTR to determine the SS pin funktion
	 * SS pin is automatic SS output
	 */
	SPI1->C1 = SPI_C1_MSTR_MASK | SPI_C1_SSOE_MASK;


	SPI1->C1 &= ~SPI_C1_CPHA_MASK;
	SPI1->C1 &= ~SPI_C1_CPOL_MASK;

	/*
	 * Sets the prescaler and bit rate
	 * SPPR: Prescaler is sets to divisor with 5
	 * BR: Bit rate is set to 4
	 */
	SPI1->BR = SPI_BR_SPPR(4) | SPI_BR_SPR(1);

	/*
	 * Set interrups for SPI when receive data
	 * Read register SPI1_S
	 *
	 * interrupts only activ on SPI enabled (SPE bit set)
	 * Interrupts for receiver: Enable SPIE
	 * SPIE enabler SPRF and MODF
	 * Interrupt is handle by the IRQHandler
	 *
	 */
	//SPI1->C1 |= SPI_C1_SPIE_MASK;

	/*
	 * Enable SPI system again for use.
	 * Set SS pin high
	 */

	PTE->PSOR = (1 << SPI1_SS_PIN);
	SPI1->C1 |= SPI_C1_SPE_MASK;

}

void SPI_transmit(uint8_t d_out)
{
	//waits for the transmit buffer to empty
	while(!(SPI1->S & SPI_S_SPTEF_MASK))
		;
	SPI1->D = d_out; //SPRF flag is set, because the read buffer is full

}

uint8_t SPI_receive()
{
	while(!(SPI1->S & SPI_S_SPRF_MASK))
					;
		return SPI1->D;
}

uint8_t SPI_send(int wrn, uint8_t Address, uint8_t Data )
{
	uint8_t address;

		//Sets the read/write bit in the beginning of the address byte
		address = Address | (wrn << 7);
		//Set the SS pin low
		SS_T();

		//send the data
		SPI_transmit(address);
		SPI_transmit(Data);

		//delay, so the data can be transmittet before SS goes low
		while(!(SPI1->S & SPI_S_SPTEF_MASK))
				;

		//pulls SS pin high
		SS_T();

		return SPI1->D;
}

void SS_T()
{
	PTE->PTOR = (1 << SPI1_SS_PIN);
}










