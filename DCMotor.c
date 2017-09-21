	// DCMotor.c
	// Runs on LM4F120 or TM4C123
	// Use SysTick interrupts to implement a software PWM to drive
	// a DC motor at a given duty cycle.  The built-in button SW1
	// increases the speed, and SW2 decreases the speed.
	// Daniel Valvano, Jonathan Valvano
	// August 6, 2013

	/* This example accompanies the book
		 "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
		 ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
		 "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
		 ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

	 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
	 You may use, edit, run or distribute this file
	 as long as the above copyright notice remains
	 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
	 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
	 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
	 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
	 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
	 For more information about my classes, my research, and my books, see
	 http://users.ece.utexas.edu/~valvano/
	 */
	 
	//**************************************************
	//	Name: Michael Handria & Chris Lopez
	//	Date: 12/7/2016
	//	Proj: Project PHASE I & PHASE II
	//	- This project is originaly by Jonathan W. Valvano 
	//	- edited by Michael Handria & Chris Lopez
	//**************************************************

	//includes the clock counter from PLL header file
	#include "PLL.h"

	//includes all the ports in this header file
	#include "tm4c123gh6pm.h"
	#include "ADCSWTrigger.h"
	#include "Nokia5110.h"
	#include "PWM.h"
	#include <stdbool.h>
	#include <stdint.h>


	//define the total period and leds of the project
	#define period 80000
	#define H 		 24000
	#define L 		 56000

	//---------------- WILL NOT BE USED IN PROJ 3 ---------------------
	//#define leds 0x02
	//#define ledf 0x08
	//#define ledb 0x04
	//_________________________________________________________________


	// basic functions defined at end of startup.s
	void DisableInterrupts(void); 			// Disable interrupts
	void EnableInterrupts	(void);  			// Enable interrupts
	long StartCritical 		(void);    		// previous I bit, disable interrupts
	void EndCritical			(long sr);    // restore I bit to previous value
	void WaitForInterrupt	(void);  			// low power mode

	// initial function prototypes below
	void Values_Init			(void);
	void Motor_Init				(void);
	void Switch_Init			(void);
	void updatePWM				(void);


	//master function (A FUNCTION THAT INITIALIZES ALL OTHER FUNCTION)
	void INITIALIZE(void);




	// these value will hold the ADC value comming from port E
	unsigned long 	IRL, IRR;
	float 					PWM;

	//this allows calculating a percentage of the duty cycle
	unsigned long 	Duty_Cycle;
	

	// these variables will hold the value of the converted 
	// adc value from the IR sensor relative to cm
	unsigned short 	distanceL, distanceR;


	// used to calculate a filter for the distance
	unsigned long  	arrayL, arrayR;



	//------ THESE VALUES MUST BE INITIALIZED -------
	bool 						stop;				//set to FALSE

	unsigned long  	timelapse ; //set to zero
	unsigned long 	distance;   //set to 1900
	unsigned short  filter;			//set to zero
	unsigned long   maxHigh;		//set to 20,000 (50% DutyCycle)
	//--------------------------------------------------




	//---------- INITIALIZE ROBOT --------------
	//	
	//	ALL THIS FUNCTION DOES IS INITIALIZE
	//	GLOBAL VARIABLES | PORTS | LCD-DISPLAYS
	//
	//------------------------------------------
	void INITIALIZE(void){
		
				DisableInterrupts();  								// disable interrupts while initializing
				Switch_Init();
				PLL_Init();           								// bus clock at 80 MHz
				PWM0B_Init(40000, 20000);         		// initialize PWM0, 1000 Hz, 50% duty
				PWM0A_Init(40000, 20000);        			// initialize PWM0, 1000 Hz, 50% duty
				ADC0_InitSWTriggerSeq3_Ch1();         // ADC initialization PE2/AIN1
				Nokia5110_Init();											// initialize the LCD SCREEN
				Nokia5110_Clear();										// CLEARS the LCD SCREEN
				Motor_Init();
				EnableInterrupts();  									// enable after all initialization are done
	}


	//---------- INITIALIZE ROBOT --------------
	//	
	//	ALL THIS FUNCTION DOES IS INITIALIZE
	//	Stop | timelapse | distance | filter
	//	stop 			- when e-stop is pressed
	//	timelapse - refresh screen 
	//	distance	- avoid this distance
	//	filter		- filter adc signal comming in 
	//
	//------------------------------------------
	void Values_Init(void){
				stop = false;				// set to FALSE
				timelapse = 0; 			// set to zero
				distance = 1900;   	// set to 1900
				filter = 0;					// set to zero
				maxHigh = 20000;		// set to 20,000 (50%)
	}


	void Motor_Init(void){
				SYSCTL_RCGC2_R |= 0x00000008; 			 // activate clock for port D
				
				
				//forward & backward functions
				GPIO_PORTD_LOCK_R = 0x4C4F434B;
				GPIO_PORTD_AMSEL_R &= ~0x0C;      	// disable analog functionality on PD3 & PD2
				GPIO_PORTD_PCTL_R &= ~0x0000FF00;		// configure PD3 & PD2 as GPIO
				GPIO_PORTD_DIR_R |= 0x0C;    				// make PD3 & PD2 out
				GPIO_PORTD_DR8R_R |= 0x0C;    			// enable 8 mA drive on PD3 & PD2
				GPIO_PORTD_AFSEL_R = 0x00;  				// disable alt funct on PD3 & PD2
				GPIO_PORTD_DEN_R |= 0x0C;     			// enable digital I/O on PD3 & PD2
				GPIO_PORTD_DATA_R = 0x0C;   				// make PD3 & PD2 low
	}



	//---------------UPDATE THE PWM---------------------------
	//	This function will be called everytime a SysTick interupt
	//	is called.   
	//
	//	NOTE**: this function will update the pwm through hardware
	//					pwm and not systick. Also this function will also 
	//					update the pwm accordingly to the distance
	//---------------------------------------------------------
	void updatePWM(void){
					
					/*																							*\
						Due to the direction of current in the 
						hbrige, logic for the pwm can change.
						When PortA is outputting a high signal, 
						the motor is turned on by outputing a low
						signal to the motor.
		
						!IMPORTANT: this is entirely on how you wire your
												h-bridge and motor 
					\*																								*/
				
				
				//grabs the adc converted value after it gets filtered
				if(filter == 3){
					
					//--------- ALGORITHM -----------------	
					// this algorightm was derived from the concept of 1/x
					// NOTE: 1/x creates a graph that is not linear 
				
					distanceR = 24.28/(((arrayR/3)*3.3/4095.0)-2.867+0.1*24.28);
					distanceL = 24.28/(((arrayL/3)*3.3/4095.0)-2.867+0.1*24.28);
					
					//---------------------------------------
					
				}
				else{
					
					//stores the sampled values in an array
					ADC0_InSeq3(&PWM, &IRL, &IRR);
					arrayR += IRR;
					arrayL += IRL;
				}
				
				// catches if the distance goes above 70
				// the algorithm is not perfect so it will somtimes calculate to 
				// a vlaue above 70 <-- we don't want that 
				// REFER: TO DATASHEET IR SENSOR SHARP VIEW SLOPE GRAPH
				
				if(distanceR > 70) distanceR = 70;
				if(distanceL > 70) distanceL = 70;
					
				//if distance is out of range then stop the motor from running
				if(IRR <= 900 && IRL <= 900){
					GPIO_PORTF_DATA_R = 0x02;
					PWM0B_Duty(1);
					PWM0A_Duty(1);
				}
				
				
				else{
					
					// If statement will go into if the distance is too close
					// If the statment is not true then don't go throught
					// and go to the else statment instead
					
					if(IRR >= distance || IRL >= distance){
						if(IRR > IRL){
							GPIO_PORTF_DATA_R = 0x04;    // LED is blue
							
							//make left
							PWM0A_Duty((maxHigh*.25));
							PWM0B_Duty(maxHigh*.75);
							if(IRR >= 3300){
								GPIO_PORTF_DATA_R = 0x04;    // LED is blue
								
								//sharp left
								PWM0A_Duty(1);
								PWM0B_Duty(maxHigh);
							}
						}
						else {
							GPIO_PORTF_DATA_R = 0x08;  // LED is green
							
							//make right
							PWM0A_Duty((maxHigh*.75));
							PWM0B_Duty((maxHigh*.25));
							if(IRL >= 3300){
								GPIO_PORTF_DATA_R = 0x08;  // LED is green
								
								//sharp right
								PWM0A_Duty(maxHigh);
								PWM0B_Duty(1);
							}
						}


					}
					else{
						
						if(IRL > IRR){
							//make right
							PWM0A_Duty((maxHigh+500));
							PWM0B_Duty(maxHigh);
						}
						else{
							//make left
							PWM0A_Duty(maxHigh);
							PWM0B_Duty((maxHigh+500));
						}
					}
				}
				
				// this will increment the filter for the if statment
				// where your Analog Signal is being sampled
				filter ++;
	}




	//-----------------SWITCH INITIALIZE---------------------
	//	This function is to initialize the switch that will
	//	be used and the led to indicate the robot's direction
	//	and motion
	//	IN PROJECT 
	//	SW1 	= emergency stop!!
	//	SW2 	= changes avoidance mode
	//	LED G = will show when the right	
	//	LED B = will show when the left 
	//	LED R = will show when the motor is stopped			
	//--------------------------------------------------------

	void Switch_Init(void){  unsigned long volatile delay;
		
		SYSCTL_RCGC2_R |= 0x00000020; 		// activate clock for port F
		delay = SYSCTL_RCGC2_R;
		
		GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
		GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
		GPIO_PORTF_AMSEL_R = 0x00;        // disable analog function
		GPIO_PORTF_PCTL_R = 0;   					// GPIO clear bit PCTL  
		GPIO_PORTF_DIR_R = 0x0E;          // PF4,PF0 input, PF3,PF2,PF1 output   
		GPIO_PORTF_AFSEL_R = 0x00;        // no alternate function
		GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
		GPIO_PORTF_DEN_R = 0x1F;          // enable digital pins PF4-PF0  
		
		GPIO_PORTF_IS_R &= ~0x11;     		// PF4,PF0 is edge-sensitive
		GPIO_PORTF_IBE_R &= ~0x11;    		// PF4,PF0 is not both edges
		GPIO_PORTF_IEV_R &= ~0x11;    		// PF4,PF0 falling edge event
		GPIO_PORTF_ICR_R = 0x11;      		// clear flags 4,0
		GPIO_PORTF_IM_R |= 0x11;      		// arm interrupt on PF4,PF0 
		
		// priority 2
		NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; 
		
		NVIC_EN0_R = 0x40000000;      		// enable interrupt 30 in NVIC
	}


	//----------------------GPIO F HANDLER---------------------
	// this function allows us to change the mode through
	// the port F interupt 
	// SW 1 -> EMERGENCY STOP
	// SW 2 -> CHANGES THE AVOIDANCE DISTANCE 
	//					-> toggles between 20 cm & 30 cm
	//----------------------------------------------------------

	void GPIOPortF_Handler(void){ 
		
		//----- SW 1 -------\\
		
		if(GPIO_PORTF_RIS_R&0x10){
			GPIO_PORTF_ICR_R = 0x10;
			
			// toggle the stop
			if(stop){
				stop = false;
			}
			else{
				stop = true;
			}
			
			//if stop is true then change the pwm to 0 
			if(stop){
				PWM0B_Duty(1);
				PWM0A_Duty(1);
			}
		}

		//---- SW 2 ------\\
		
		if(GPIO_PORTF_RIS_R&0x01){
			GPIO_PORTF_ICR_R = 0x01;
			if(distance == 1900){
				distance = 1500;
			}
			else{
				distance = 1900;
			}
		}
	}

	int main(void){
		
		INITIALIZE();
		
		while(1){
			if(stop){
				
				//---------------------------NOKIA DISPLAY FUNCTIONALITIES ------------------------------
				//
				// This function will only display when the emergency stop button is pressed
				// it will display a red light on the board and will display "E-stop" on the 
				// lcd display (NOKIA 5110 red)
				//---------------------------------------------------------------------------------------
				GPIO_PORTF_DATA_R = 0x02;
						
				//clear the lcd before writing to it
				Nokia5110_Clear();										
				
				//write to LCD
				Nokia5110_OutString("E-stop");
			}
			else{
				
				//updates the pwm functionality 
				updatePWM();
				
				
				//---------------------------NOKIA DISPLAY FUNCTIONALITIES ------------------------------
				//
				// NOTE: timelapse allows a delay before reset
				// 1.) first row will display right value in cm
				// 2.) second row will display left value in cm
				// 3.) third row will display the mode the current
				// 4.) fifth row will display the duty cycle 
				//---------------------------------------------------------------------------------------
				if(timelapse == 6500){
					timelapse = 0;
					
					//calculates the duty cycle in percentage to be
					Duty_Cycle = (PWM >= 2700) 								? 100: 
											 (PWM >= 1400 && PWM <= 2300) ?  50:
																											 35;
					
					maxHigh 	 = (Duty_Cycle == 100)									? 40000:
											 (Duty_Cycle == 50)										? 20000:
																															18000;
					
					//set cursor to origin point row 0 column 0
					Nokia5110_SetCursor(0,0);
				

					
					// display distance of right 
					Nokia5110_OutString("R:");
					Nokia5110_OutUDec(distanceR);
					Nokia5110_OutString("cm");
					Nokia5110_SetCursor(0,1);
					
					// display distance of left
					Nokia5110_OutString("L:");
					Nokia5110_OutUDec(distanceL);
					Nokia5110_OutString("cm");
					
					
					// display how far it will avoid
					Nokia5110_SetCursor(0,2);
					Nokia5110_OutString("avoid: ");
					
					// if else statement to display what current mode is 
					if(distance == 1900){
						Nokia5110_OutString("20cm");
					}
					else{
						Nokia5110_OutString("30cm");
					}
					
					//display duty cycle 
					Nokia5110_SetCursor(0,4);
					Nokia5110_OutString("Duty: ");
					Nokia5110_OutUDec(Duty_Cycle);
					Nokia5110_OutString("%");
				}
				timelapse++;
			}
		}
	}
