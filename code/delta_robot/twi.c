/*

		TWI MAIN FILE
		twi.c
*/

#include "twi.h"

void twi_init()
{
	DDRC = 0x03;								//--- PORTC Last two bit as Output
	PORTC = 0x03;
	
	TWCR &= ~(1<<TWEN);							//--- Diable TWI
	TWBR = BITRATE(TWSR = 0x00);				//--- Bit rate with prescaler 4
	TWCR = (1<<TWEN);							//--- Enable TWI
	_delay_us(10);								//--- Delay
}

/* Function to Send Start Condition */

void twi_start()
{
	TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//--- Start Condition as per Datasheet
	while(!(TWCR & (1<<TWINT)));				//--- Wait till start condition is transmitted to Slave
	while(TW_STATUS != TW_START);				//--- Check for the acknowledgment 0x08 = TW_START
}

/* Function to Send Slave Address for Write operation */

void twi_write_cmd(unsigned char address)
{
	TWDR=address;								//--- SLA Address and write instruction
	TWCR=(1<<TWINT)|(1<<TWEN);					//--- Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT)));				//--- Wait till complete TWDR byte transmitted to Slave
	while(TW_STATUS != TW_MT_SLA_ACK);			//--- Check for the acknowledgment
}

/* Function to Send Data to Slave Device  */

void twi_write_dwr(unsigned char data)
{
	TWDR=data;									//--- Put data in TWDR
	TWCR=(1<<TWINT)|(1<<TWEN);					//--- Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT)));				//--- Wait till complete TWDR byte transmitted to Slave
	while(TW_STATUS != TW_MT_DATA_ACK);			//--- Check for the acknowledgment
}

/* Function to Send Stop Condition */

void twi_stop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);		//--- Stop Condition as per Datasheet
}

/* Function to Send Repeated Start Condition */


void twi_repeated_start()
{
	TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//--- Repeated Start Condition as per Datasheet
	while(!(TWCR & (1<<TWINT)));				//--- Wait till restart condition is transmitted to Slave
	while(TW_STATUS != TW_REP_START);			//--- Check for the acknowledgment
}


/* Function to Send Read Acknowledgment */

char twi_read_ack()
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);		//--- Acknowledgment Condition as per Datasheet
	while (!(TWCR & (1<<TWINT)));				//--- Wait until Acknowledgment Condition is transmitted to Slave
	while(TW_STATUS != TW_MR_DATA_ACK);			//--- Check for Acknowledgment 						
	return TWDR;								//--- Return received data from Slave
}

/* Function to Send Read No Acknowledgment */

char twi_read_nack()
{
	TWCR=(1<<TWEN)|(1<<TWINT);					//--- No Acknowledgment Condition as per Datasheet
	while (!(TWCR & (1<<TWINT)));				//--- Wait until No Acknowledgment Condition is transmitted to Slave
	while(TW_STATUS != TW_MR_DATA_NACK);		//--- Check for Acknowledgment
	return TWDR;								//--- Return received data
}

/****** END of Program ******/