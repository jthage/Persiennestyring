/*
 * SPI.h
 *
 *  Created on: 9. nov. 2018
 *      Author: andreas
 */

/*
 *
 */

#include <MKL25Z4.h>

// Pin setup the SPI1 communication
//Pin setup on Port E

#define SPI1_SS_PIN 4
#define SPI1_SCK_PIN 2
#define SPI1_MISO_PIN 1
#define SPI1_MOSI_PIN 3

#define SS_L

#ifndef SPI_H_
#define SPI_H_



void SPI_Init();
void SPI_transmit( uint8_t d_out);
uint8_t SPI_recieve();

void SS_T();
uint8_t SPI_send(int wrn, uint8_t Address, uint8_t Data );

#endif /* SPI_H_ */
