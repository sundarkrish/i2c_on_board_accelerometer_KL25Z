/*
 * accel.c
 *
 *  Created on: Mar 21, 2020
 *      Author: SUNDAR
 */


#include <MKL25Z4.H>
#include "accel.h"
#include "KL25Z_delay.h"
int lock_detect=0;
int i2c_lock=0;

uint8_t flag = 1;
uint8_t data[6]= {0};
int16_t acc_X=0, acc_Y=0, acc_Z=0;
float roll=0.0, pitch=0.0;
int8_t count = -1; // Number of bytes to be received
int i;
int16_t temp[3];
//mma data ready
extern uint32_t DATA_READY;

//init i2c0
void i2c_init(void)
{
	//clock i2c peripheral and port E
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
	SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK);

	//set pins to I2C function
	PORTE->PCR[24] |= PORT_PCR_MUX(5);
	PORTE->PCR[25] |= PORT_PCR_MUX(5);

	//set to 100k baud
	//baud = bus freq/(scl_div+mul)
 	//~400k = 24M/(64); icr=0x12 sets scl_div to 64

 	I2C0->F = (I2C_F_ICR(0x10) | I2C_F_MULT(0));

	//enable i2c and set to master mode
	I2C0->C1 |= (I2C_C1_IICEN_MASK);

	// Select high drive mode
	I2C0->C2 |= (I2C_C2_HDRS_MASK);
}


void i2c_busy(void){
	// Start Signal
	lock_detect=0;
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C_TRAN;
	I2C_M_START;
	I2C0->C1 |=  I2C_C1_IICEN_MASK;
	// Write to clear line
	I2C0->C1 |= I2C_C1_MST_MASK; /* set MASTER mode */
	I2C0->C1 |= I2C_C1_TX_MASK; /* Set transmit (TX) mode */
	I2C0->D = 0xFF;
	while ((I2C0->S & I2C_S_IICIF_MASK) == 0U) {
	} /* wait interrupt */
	I2C0->S |= I2C_S_IICIF_MASK; /* clear interrupt bit */


							/* Clear arbitration error flag*/
	I2C0->S |= I2C_S_ARBL_MASK;


							/* Send start */
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C0->C1 |= I2C_C1_TX_MASK; /* Set transmit (TX) mode */
	I2C0->C1 |= I2C_C1_MST_MASK; /* START signal generated */

	I2C0->C1 |= I2C_C1_IICEN_MASK;
							/*Wait until start is send*/

							/* Send stop */
	I2C0->C1 &= ~I2C_C1_IICEN_MASK;
	I2C0->C1 |= I2C_C1_MST_MASK;
	I2C0->C1 &= ~I2C_C1_MST_MASK; /* set SLAVE mode */
	I2C0->C1 &= ~I2C_C1_TX_MASK; /* Set Rx */
	I2C0->C1 |= I2C_C1_IICEN_MASK;


								/* wait */
							/* Clear arbitration error & interrupt flag*/
	I2C0->S |= I2C_S_IICIF_MASK;
	I2C0->S |= I2C_S_ARBL_MASK;
	lock_detect=0;
	i2c_lock=1;
}


void i2c_wait(void) {
	lock_detect = 0;
	while(((I2C0->S & I2C_S_IICIF_MASK)==0) & (lock_detect < 200)) {
		lock_detect++;
	}
	if (lock_detect >= 200)
		i2c_busy();
	I2C0->S |= I2C_S_IICIF_MASK;
}

//send start sequence
void i2c_start()
{
	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
}

//send device and register addresses

void i2c_read_setup(uint8_t dev, uint8_t address)
{
	uint8_t dummy;

	I2C0->D = dev;			  /*send dev address	*/
	I2C_WAIT							/*wait for completion */

	I2C0->D =  address;		/*send read address	*/
	I2C_WAIT							/*wait for completion */


	I2C_M_RSTART;				   /*repeated start */
	I2C0->D = (dev|0x1);	 /*send dev address (read)	*/
	I2C_WAIT							 /*wait for completion */

	I2C_REC;						   /*set to receive mode */
	ACK;								/*ACK after read	*/
	enable_i2c_interrupt();
	dummy = I2C0->D;				/*dummy read	*/
}

//read a byte and ack/nack as appropriate
// #pragma no_inline
uint8_t i2c_repeated_read(uint8_t isLastRead)
{
	uint8_t data;

	lock_detect = 0;

	if(isLastRead)	{
		NACK;								/*set NACK after read	*/
	} else	{
		ACK;								/*ACK after read	*/
	}

	data = I2C0->D;				/*dummy read	*/
	I2C_WAIT							/*wait for completion */

	if(isLastRead)	{
		I2C_M_STOP;					/*send stop	*/
	}
	data = I2C0->D;				/*read data	*/

	return  data;
}

//using 7bit addressing writes a byte data to dev:address

void i2c_write_byte(uint8_t dev, uint8_t address, uint8_t data)
{

	I2C_TRAN;							/*set to transmit mode */
	I2C_M_START;					/*send start	*/
	I2C0->D = dev;			  /*send dev address	*/
	I2C_WAIT						  /*wait for ack */

	I2C0->D =  address;		/*send write address	*/
	I2C_WAIT

	I2C0->D = data;				/*send data	*/
	I2C_WAIT
	I2C_M_STOP;

}





//initializes mma8451 sensor
//i2c has to already be enabled
int init_mma()
{
	//set active mode, 14 bit samples and 800 Hz ODR
	i2c_write_byte(MMA_ADDR, REG_CTRL1, 0x01);

	return 1;
}

void enable_i2c_interrupt(void)
{

	__disable_irq();

	I2C0->C1 |= 0x40; // enable interrupt bit
	NVIC_ClearPendingIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_SetPriority(I2C0_IRQn, 3);

	__enable_irq();


}

void disable_i2c_interrupt(void)
{



	I2C0->C1 &= ~0x40; // disable interrupt bit
	NVIC_DisableIRQ(I2C0_IRQn);

	__disable_irq();


}

void read_full_xyz()
{
	//int i;
	//int16_t temp[3];

	i2c_start();
	i2c_read_setup(MMA_ADDR , REG_XHI);

	// Read five bytes in repeated mode
	/*for( i=0; i<5; i++)	{
		data[i] = i2c_repeated_read(0);
	}
	// Read last byte ending repeated mode
	data[i] = i2c_repeated_read(1);

	for ( i=0; i<3; i++ ) {
		temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
	}

	// Align for 14 bits
	acc_X = temp[0]/4;
	acc_Y = temp[1]/4;
	acc_Z = temp[2]/4;*/
}

void I2C0_IRQHandler(void)
{

	I2C0->S |= I2C_S_IICIF_MASK; // clear interrupt pending bit

	flag = 0;

	//I2C_REC;						   /*set to receive mode */
	count++;
	//uint8_t dummy;

	lock_detect = 0;

	if(count == 4)	{
		NACK;								/*set NACK after read	*/
	} else	{
		ACK;								/*ACK after read	*/
	}

	//dummy = I2C0->D;				/*dummy read	*/
	//I2C_WAIT							/*wait for completion */

	if(count == 5)	{
		I2C_M_STOP;					/*send stop	*/
	}
	data[count] = I2C0->D;				/*read data	*/

	if (count == 5)
	{
		I2C0->C1 &= ~0x40; // disable interrupt bit
		flag = 1;
		NVIC_DisableIRQ(I2C0_IRQn);
		count = -1;
		for ( i=0; i<3; i++ ) {
			temp[i] = (int16_t) ((data[2*i]<<8) | data[2*i+1]);
		}

		// Align for 14 bits
		acc_X = temp[0]/4;
		acc_Y = temp[1]/4;
		acc_Z = temp[2]/4;

	}



}




