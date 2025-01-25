/*
 * delta_robot.c
 *
 * Created: 5/10/2024 10:04:49 PM
 * Author : Ghaith Zahda
 */ 
#define F_CPU 8000000UL	
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "twi.h"
#include "twi_lcd.h"

#define LED_KEYPAD PD2
#define LED_JOYSTICK PB0
#define DIR1 PB4
#define DIR2 PD3
#define DIR3 PD6
#define StepMotor1 PB3
#define StepMotor2 PD4
#define StepMotor3 PD7
#define Z_Inc PA3
#define Z_Dec PA4
#define xMax PA5
#define yMax PA6
#define zMax PA7


// -------------------Global Variables--------------------
int step1 = 0, step2 = 0, step3 = 0;  // The number of steps to move
int degreesMotor1 = 0, degreesMotor2 = 0, degreesMotor3 = 0 // Angle motors=(degreesMotor/2)*0.1125
	,speedMotor1,speedMotor2,speedMotor3; // Speed motors
int theta_1=9999,theta_2=9999,theta_3=9999; // Target angle
float myTime // The time required for the longest distance between motors
	,Link=80,l=135.93,a=55.4,b=-47.98,c=-27.7; // Robot Variables
char dataLcd[10]; // Data sent to the screen
float E1,E2,E3,F1,F2,F3,G1,G2,G3,t1,t2,t3 // Variables of robotic equations
	,X = 0, Y = 0, Z = 0,xx=0,yy=0,zz=-165; // Coordinates in millimeters
int  ADC_NUM=0 , adc_data[8]; // ADC data
long Keypad , counterKeypad=0 , filterKeypad=0 , ADCKeypad=1024; // Keypad Variables
long  xNum[2]={99,99} , yNum[2]={99,99} , zNum[3]={99,99,99} , numLocation=0 , modeTybe=0 , Keypadnum=0; // Keypad Variables
long mode=1 , decFlagx=0 , decFlagy=0 , ix=0 , iy=0 , iz=0 , go=0 , counterX=0 , counterY=0 , counterZ=0 ; // Keypad Variables
long  counterkey=0;
long lcdDelay=0,flag;

//-------------------------function-----------------------
void moveMotors(float x, float y, float z, int speed);

//------------------interrupt ADC---------------------
ISR(ADC_vect) {
	adc_data[ADC_NUM]=ADCW; //Read the AD conversion result
	ADC_NUM++;  //Select next ADC input
	if (ADC_NUM==3) ADC_NUM=0;
	if (ADC_NUM==1){
		counterKeypad++;
		filterKeypad+=adc_data[0];
		if(counterKeypad>=256) {
			ADCKeypad=filterKeypad>>8;
			filterKeypad=0;
			counterKeypad=0;
			if(ADCKeypad<20) Keypad=1;
			else if(ADCKeypad>25 && ADCKeypad<55)  Keypad=2;
			else if(ADCKeypad>55 && ADCKeypad<70)  Keypad=3;
			else if(ADCKeypad>85 && ADCKeypad<95)  Keypad=11;//x
			else if(ADCKeypad>100 && ADCKeypad<140)  Keypad=4;
			else if(ADCKeypad>140 && ADCKeypad<160)  Keypad=5;
			else if(ADCKeypad>170 && ADCKeypad<190)  Keypad=6;
			else if(ADCKeypad>195 && ADCKeypad<210)  Keypad=12;//y
			else if(ADCKeypad>225 && ADCKeypad<240)  Keypad=7;
			else if(ADCKeypad>245 && ADCKeypad<260)  Keypad=8;
			else if(ADCKeypad>265 && ADCKeypad<275)  Keypad=9;
			else if(ADCKeypad>280 && ADCKeypad<300)  Keypad=13;//z
			else if(ADCKeypad>301 && ADCKeypad<320)  Keypad=14;//switch
			else if(ADCKeypad>320 && ADCKeypad<335)  Keypad=0;
			else if(ADCKeypad>335 && ADCKeypad<350)  Keypad=15;//-
			else if(ADCKeypad>350 && ADCKeypad<365)  Keypad=16;//go
			else   Keypad=99;
		}
	}
	ADMUX=ADC_NUM;
	_delay_us(10);
	ADCSRA |= (1 << ADSC);
}

//------------------interrupt timer0---------------------
ISR(TIMER0_COMP_vect) {
	if (degreesMotor1 != theta_1) {
		if ((PINB &(1 << DIR1))) degreesMotor1++;
		else degreesMotor1--;
		PORTB ^= (1 << StepMotor1);
	}
}

//------------------interrupt timer1-------------------
ISR(TIMER1_OVF_vect) {
	TCNT1=255-OCR1A+65280;
	if (degreesMotor2 != theta_2) {
		if ((PIND &(1 << DIR2))) degreesMotor2++;
		else degreesMotor2--;
		PORTD ^= (1 << StepMotor2);
	}

}


//------------------interrupt timer2------------------
ISR(TIMER2_COMP_vect) {
	if (degreesMotor3 != theta_3) {
		if ((PIND &(1 << DIR3))) degreesMotor3++;
		else degreesMotor3--;
		PORTD ^= (1 << StepMotor3);
	}
}
int main(void)
{
//-------------------LCD initial-----------------------	
	twi_lcd_init();
	_delay_ms(500);

//-------------------Registers-----------------------
	TCNT0 = 0;
	TCCR0 = (1 << WGM01) | (1 << CS02) | (1 << CS00);
	TCNT1 = 65280;
	TCCR1B = (1 << CS10) | (1 << CS12);
	TCNT2 = 0;
	TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS20) | (1 << CS21);
	OCR0 = 50;
	OCR1A = 50;
	OCR2 = 50;
	DDRB = (1 << DIR1) | (1 << StepMotor1) | (1 << LED_JOYSTICK);
	DDRD = (1 << DIR2) | (1 << StepMotor2) | (1 << DIR3) | (1 << StepMotor3) | (1 << LED_KEYPAD);
	PORTA = (1 << Z_Dec) | (1 << Z_Inc);
	
	
	// Enable Interrupts
	TIMSK = (1 << OCIE0) | (1 << TOIE1) | (1 << OCIE2);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX = 0;
	ADCSRA |= (1 << ADSC);
	sei();
//----------------------GO to Zero point-----------------------
	PORTB |= (1 << DIR1);
	PORTD  = (1 << DIR2) | (1 << DIR3);
	twi_lcd_cmd(0xC0);
	twi_lcd_msg("     Go  to     ");
	twi_lcd_cmd(0x90);
	twi_lcd_msg(" limit switches ");
	while(degreesMotor1!=1702 || degreesMotor2!=1702 || degreesMotor3!=1702) {
		if(PINA &(1 << xMax)) {
			theta_1=1702;
			degreesMotor1=1702;
		}
		if(PINA &(1 << yMax)) {
			theta_2=1702;
			degreesMotor2=1702;
		}
		if(PINA &(1 << zMax)) {
			theta_3=1702;
			degreesMotor3=1702;
		}
	}
	twi_lcd_cmd(0x01);
	_delay_ms(2);
	twi_lcd_cmd(0xC0);
	twi_lcd_msg("     Go  to     ");
	twi_lcd_cmd(0x90);
	twi_lcd_msg("   Zero point   ");
	_delay_ms(500);
	X = xx;
	Y = yy;
	Z = zz;
	moveMotors(X, Y, Z, 10);
	while (step1!=0 || step2!=0 || step3!=0) moveMotors(X, Y, Z, 10);
	twi_lcd_cmd(0x01);
	_delay_ms(2);
	
//----------------------LCD begin------------------------------
	twi_lcd_cmd(0x80);
	twi_lcd_msg("x=");
	twi_lcd_cmd(0x87);
	twi_lcd_msg("t1=");
	twi_lcd_cmd(0xC0);
	twi_lcd_msg("y=");
	twi_lcd_cmd(0xC7);
	twi_lcd_msg("t2=");
	twi_lcd_cmd(0x90);
	twi_lcd_msg("z=");
	twi_lcd_cmd(0x97);
	twi_lcd_msg("t3=");
	twi_lcd_cmd(0xd0);
	twi_lcd_msg("    Joystick");
	PORTB |= (1 << LED_JOYSTICK);
    while (1) 
    {
		if(0) {
			goo:
			twi_lcd_cmd(0x8A);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0x8A);
			dtostrf((degreesMotor1/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0xCA);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0xCA);
			dtostrf((degreesMotor2/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0x9A);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0x9A);
			dtostrf((degreesMotor3/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
		}
		if(Keypad==14) {
			counterkey++;
			if(mode==-1) flag=25000;
			else flag=50;
			if(counterkey == flag ) {
				counterkey=25001;
				mode *= -1;	
				twi_lcd_cmd(0xd0);
				twi_lcd_msg("            ");
				twi_lcd_cmd(0xd0);
				if(mode==1) {
					twi_lcd_msg("    Joystick");
					PORTD &= ~(1 << LED_KEYPAD);
					PORTB |= (1 << LED_JOYSTICK);
					twi_lcd_cmd(0x0c);
				}
				if(mode==-1){ 
					twi_lcd_msg("     Keypad");
					PORTD |= (1 << LED_KEYPAD);
					PORTB &= ~(1 << LED_JOYSTICK);
					twi_lcd_cmd(0x82);
					twi_lcd_msg("    ");
					twi_lcd_cmd(0xC2);
					twi_lcd_msg("    ");
					twi_lcd_cmd(0x92);
					twi_lcd_msg("    ");
					twi_lcd_cmd(0x92);
					twi_lcd_msg("-");
					modeTybe=0;
				}
			}
		}
		else if(Keypad==11 && mode==-1) {
			counterkey++;
			if(counterkey == 10000 ) {
				twi_lcd_cmd(0x0E);
				modeTybe=11;
				numLocation=0x83;
				ix=0;
				if(decFlagx==0) {
					twi_lcd_cmd(0x82);
					twi_lcd_msg(" ");
				}
				if(decFlagx==1) {
					twi_lcd_cmd(0x82);
					twi_lcd_msg("-");
				}
				twi_lcd_cmd(numLocation);
				twi_lcd_msg("  ");
				twi_lcd_cmd(numLocation);
				
			}
		}
		else if(Keypad==12 && mode==-1) {
			counterkey++;
			if(counterkey == 10000 ) {
				twi_lcd_cmd(0x0E);
				modeTybe=12;
				numLocation=0xC3;
				iy=0;
				if(decFlagy==0) {
					twi_lcd_cmd(0xc2);
					twi_lcd_msg(" ");
				}
				if(decFlagy==1) {
					twi_lcd_cmd(0xc2);
					twi_lcd_msg("-");
				}
				twi_lcd_cmd(numLocation);
				twi_lcd_msg("  ");
				twi_lcd_cmd(numLocation);
				if(decFlagy==0) {
					twi_lcd_cmd(0xc2);
					twi_lcd_msg(" ");
				}
				if(decFlagy==1) {
					twi_lcd_cmd(0xc2);
					twi_lcd_msg("-");
				}
			}
		}
		else if(Keypad==13 && mode==-1) {
			counterkey++;
			if(counterkey == 10000 ) {
				twi_lcd_cmd(0x0E);
				modeTybe=13;
				numLocation=0x93;
				iz=0;
				twi_lcd_cmd(numLocation);
				twi_lcd_msg("    ");
				twi_lcd_cmd(numLocation);
			}
		}
		else if(Keypad!=99 && Keypad!=11 && Keypad!=12 && Keypad!=13 && Keypad!=14 && Keypad!=16 && mode==-1 && modeTybe==11) {
			counterkey++;
			if(counterkey == 10000 ) {
				if(numLocation==0x83) {
					twi_lcd_cmd(numLocation);
					twi_lcd_msg("  ");
					xNum[1]=99;
				}
				if(Keypad==15 && decFlagx==0) {
					decFlagx=1;
					twi_lcd_cmd(0x82);
					twi_lcd_msg("-");
				}
				else if(Keypad==15 && decFlagx==1) {
					decFlagx=0;
					twi_lcd_cmd(0x82);
					twi_lcd_msg(" ");
				}
				else {
					twi_lcd_cmd(numLocation);
					xNum[ix]=Keypad;
					ix++;
					itoa(Keypad,dataLcd,10);
					twi_lcd_msg(dataLcd);
					numLocation++;
					if(numLocation==0x85) numLocation=0x83;
					if(ix==2) ix=0;
					twi_lcd_cmd(numLocation);
				}
			}
		}
		else if(Keypad!=99 && Keypad!=11 && Keypad!=12 && Keypad!=13 && Keypad!=14 && Keypad!=16 && mode==-1 && modeTybe==12) {
			counterkey++;
			if(counterkey ==10000 ) {
				if(numLocation==0xc3) {
					twi_lcd_cmd(numLocation);
					twi_lcd_msg("  ");
					yNum[1]=99;
				}
				if(Keypad==15 && decFlagy==0) {
					decFlagy=1;
					twi_lcd_cmd(0xc2);
					twi_lcd_msg("-");
				}
				else if(Keypad==15 && decFlagy==1) {
					decFlagy=0;
					twi_lcd_cmd(0xc2);
					twi_lcd_msg(" ");
				}
				else {
				twi_lcd_cmd(numLocation);
				yNum[iy]=Keypad;
				itoa(Keypad,dataLcd,10);
				twi_lcd_msg(dataLcd);
				numLocation++;
				iy++;
				if(numLocation==0xC5) numLocation=0xC3;
				if(iy==2) iy=0;
				twi_lcd_cmd(numLocation);
			}
			}
		}
		else if(Keypad!=99 && Keypad!=11 && Keypad!=12 && Keypad!=13 && Keypad!=14 && Keypad!=15 && Keypad!=16 && mode==-1 && modeTybe==13) {
			counterkey++;
			if(counterkey ==10000 ) {
				if(numLocation==0x93) {
					twi_lcd_cmd(numLocation);
					twi_lcd_msg("   ");
					zNum[1]=99;
					zNum[2]=99;
				}
				twi_lcd_cmd(numLocation);
				zNum[iz]=Keypad;
				itoa(Keypad,dataLcd,10);
				twi_lcd_msg(dataLcd);
				numLocation++;
				iz++;
				if(numLocation==0x96) numLocation=0x93;
				if(iz==3) iz=0;
				twi_lcd_cmd(numLocation);
			}
		}
		else if(Keypad==16 && mode==-1) {
			counterkey++;
			if(counterkey ==10000 ) {
			if((xNum[0]==99 && xNum[1]==99)||(xNum[0]==0 && xNum[1]==0)||(xNum[0]==0 && xNum[1]==99)) xx=0;
			else if(xNum[1]==99) xx=xNum[0];
			else if((xNum[0]==0 && xNum[1]!=99)) xx=xNum[1];
			else xx=xNum[0]*10+xNum[1];
			if(decFlagx==1) xx*=-1;
			if (xx <= -40) xx = -40;
			else if (xx >= 40) xx = 40;
			
			if((yNum[0]==99 && yNum[1]==99)||(yNum[0]==0 && yNum[1]==0)||(yNum[0]==0 && yNum[1]==99)) yy=0;
			else if(yNum[1]==99) yy=yNum[0];
			else yy=yNum[0]*10+yNum[1];
			if(decFlagy==1) yy*=-1;
			if (yy <= -40) yy = -40;
			else if (yy >= 40) yy = 40;
			
			if((zNum[0]==99 && zNum[1]==99 && zNum[2]==99)||(zNum[0]==0 && zNum[1]==0 && zNum[2]==0)||(zNum[0]==0 && zNum[1]==99)||(zNum[0]==0 && zNum[1]==0 && zNum[2]==99)) zz=0;
			else if(zNum[1]==99 && zNum[2]==99) zz=zNum[0];
			else if(zNum[2]==99) zz=zNum[0]*10+zNum[1];
			else zz=zNum[0]*100+zNum[1]*10+zNum[2];
			zz *=-1;
			if(zz>=-85) zz=-85;
			else if(zz<=-165) zz=-165;
			X = xx;
 			Y = yy;
 			Z = zz;
			twi_lcd_cmd(0x0c);
			go=1;
		}
		}
		else if(Keypad==99) {
			counterkey=0;
		}
		if(mode==1 || go==1) {
			lcdDelay++;
			if(lcdDelay>=20) {
			twi_lcd_cmd(0x82);
			twi_lcd_msg("    ");
			twi_lcd_cmd(0x82);
			itoa(xx,dataLcd,10);
			twi_lcd_msg(dataLcd);	
			twi_lcd_cmd(0x8A);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0x8A);
			dtostrf((degreesMotor1/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0xC2);
			twi_lcd_msg("    ");
			twi_lcd_cmd(0xC2);
			itoa(yy,dataLcd,10);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0xCA);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0xCA);
			dtostrf((degreesMotor2/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0x92);
			twi_lcd_msg("    ");
			twi_lcd_cmd(0x92);
			itoa(zz,dataLcd,10);
			twi_lcd_msg(dataLcd);
			twi_lcd_cmd(0x9A);
			twi_lcd_msg("      ");
			twi_lcd_cmd(0x9A);
			dtostrf((degreesMotor3/2)*0.1125, 5,2, dataLcd);
			twi_lcd_msg(dataLcd);
			lcdDelay=0;
			}
			if (adc_data[1] < 450) {
				xx += 0.2;
				if (xx >= 40) xx = 40;
			}
			else if (adc_data[1] > 650) {
				xx -= 0.2;
				if (xx <= -40) xx = -40;
			}
			if (adc_data[2] < 450) {
				yy += 0.2;
				if (yy >= 40) yy = 40;
			}
			else if (adc_data[2] > 650) {
				yy -= 0.2;
				if (yy <= -40) yy = -40;
			}
			if((PINA &(1 << Z_Inc))==0) {
				counterZ++;
				if(counterZ>=1) {
				zz+=0.3;
				if(zz>=-85) zz=-85;
				}
			}
			else if((PINA &(1 << Z_Dec))==0)
			{
				counterZ++;
				if(counterZ>=1) {
				zz-=0.3;
				if(zz<=-165) zz=-165;
				}
			}
			else counterZ=0;
			X = xx;
			Y = yy;
			Z = zz;
			moveMotors(X, Y, Z, mode==1?19:8);
			if(step1==0 && step2==0 && step3==0) {
				go=0;
				goto goo;
			}
			}
	}
}

void moveMotors(float x, float y, float z, int speed) {

	E1=2*Link*(y+a);
	F1=2*z*Link;
	G1= pow(x,2)+ pow(y,2) + pow(z,2) + pow(a,2) + pow(Link,2) + (2*y*a)-pow(l,2);
	E2= -Link * ((sqrt(3) * (x + b)) + y + c);
	F2= 2*z*(Link);
	G2= pow(x,2) + pow(y,2) + pow(z,2) + pow(b,2) + pow(c,2) + pow(Link,2) + (2*((x*b) + y*c)) - pow(l,2);
	E3= Link * ((sqrt(3) * (x - b))  -y -c);
	F3= 2*z*(Link);
	G3= pow(x,2) + pow(y,2) + pow(z,2) + pow(b,2) + pow(c,2) + pow(Link,2) + (2*((-x*b) + y*c)) - pow(l,2);
	double val1 = pow(E1, 2) + pow(F1, 2)  - pow(G1, 2);
	double val2 = pow(E2, 2) + pow(F2, 2)  - pow(G2, 2);
	double val3 = pow(E3, 2) + pow(F3, 2)  - pow(G3, 2);
	if (val1 < 0 ||val2 < 0 ||val3 < 0) goto err;
	t1= (-F1 -  sqrt(val1)) / (G1 - E1);
	t2= (-F2 -  sqrt(val2)) / (G2 - E2);
	t3= (-F3 -  sqrt(val3)) / (G3 - E3);
	err:
	theta_1 =2*((2 * ((atan(t1)*180)/3.141592654))/0.1125);
	theta_2 =2*((2 * ((atan(t2)*180)/3.141592654))/0.1125);
	theta_3 =2*((2 * ((atan(t3)*180)/3.141592654))/0.1125);


	step1 = theta_1 - degreesMotor1;
	step2 = theta_2 - degreesMotor2;
	step3 = theta_3 - degreesMotor3;

	if (step1 < 0) {
		PORTB &= ~(1 << DIR1);
		step1 *= -1;
	}
	else PORTB |= (1 << DIR1);

	if (step2 < 0) {
		PORTD &= ~(1 << DIR2);
		step2 *= -1;
	}
	else PORTD |= (1 << DIR2);

	if (step3 < 0) {
		PORTD &= ~(1 << DIR3);
		step3 *= -1;
	}
	else PORTD |= (1 << DIR3);
	if (step1!=0 && step2!=0 && step3!=0){

		if (step1 >= step2 && step1 >= step3) {
			myTime = step1 * (1 / (8000000.0 / (2 * 1024.0 * (speed + 1))));
			speedMotor1 = speed;
			speedMotor2 = ((myTime * 8000000.0) / (2 * 1024.0 * step2)) - 1;
			speedMotor3 = ((myTime * 8000000.0) / (2 * 1024.0 * step3)) - 1;
			if (speedMotor2 >= 255) speedMotor2 = 255; // Max low speed
			if (speedMotor3 >= 255) speedMotor3 = 255; // Max low speed
		}
		else if (step2 >= step1 && step2 >= step3) {
			myTime = step2 * (1 / (8000000.0 / (2 * 1024.0 * (speed + 1))));
			speedMotor2 = speed;
			speedMotor1 = ((myTime * 8000000.0) / (2 * 1024.0 * step1)) - 1;
			speedMotor3 = ((myTime * 8000000.0) / (2 * 1024.0 * step3)) - 1;
			if (speedMotor1 >= 255) speedMotor1 = 255; // Max low speed
			if (speedMotor3 >= 255) speedMotor3 = 255; // Max low speed
		}
		else if (step3 >= step2 && step3 >= step1) {
			myTime = step3 * (1 / (8000000.0 / (2 * 1024.0 * (speed + 1))));
			speedMotor3 = speed;
			speedMotor1 = ((myTime * 8000000.0) / (2 * 1024.0 * step1)) - 1;
			speedMotor2 = ((myTime * 8000000.0) / (2 * 1024.0 * step2)) - 1;
			if (speedMotor2 >= 255) speedMotor2 = 255; // Max low speed
			if (speedMotor1 >= 255) speedMotor1 = 255; // Max low speed
		}
		OCR0 = speedMotor1;
		OCR1A = speedMotor2;
		OCR2 = speedMotor3;
	}
}

