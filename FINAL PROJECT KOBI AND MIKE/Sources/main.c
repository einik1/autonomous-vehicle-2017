#include "TFC.h"
#include <math.h>
#include <string.h>
#define MUDULO_REGISTER  18750 //
#define MUDULO_REGISTER2  50000 //
#define STEP MUDULO_REGISTER/10 // 
#define CM15 3090
#define CM25 2800
#define CM35 2200
#define CM45 1800
#define CM125 1300

uint8_t Temp, adjustedSpeed = 0;
long distanceToObs = 0;
int tempMoveCounter = 0, leftClick = 0, rightClick = 0, tempMoveCounterY = 0,
		sonicT2Arr = 0, sonicT1Arr = 0, toStop = 0;
short sonicTOF1 = 0, sonicTOF2 = 0, servoDir1 = 1, servoDir2 = -1, casePIT = 0,
		PITcount = 0, foundObs = 0, toTurn = 0, adjustCounter = 0, sonicToggle =
				0, obsCounter = 0, distCounter = 0, validObs = 0,
		finalValidCounter = 0, toMove = 100, doubleTurn = 0, irCompare = 0, Y =
				0, X = 0, c = 0, rightIR = 1, adcFlag = 0, didITurnRight = 0;
uint8_t sonicState1 = 0, sonicState2 = 0, measPit = 0; // 
int StateOfOp = 0, t11 = 0, t12 = 0, t21 = 0, t22 = 0, toggle1 = 0, toggle2 = 0; // for old FTM
int OldTime1 = 0, NewTime1 = 0, OldTime2 = 0, NewTime2 = 0, TOF1 = 0, TOF2 = 0,
		TOF1old = 0, TOF2old = 0;
int servo1, servo2, distArr[2][15], obsLoc[4][2];
extern short isValid1, isValid2;
double EncoderSensing(int engine);
void turnRight(), turnLeft(), testFunc(), measureIRRight(), measureIRLeft();
void HowMuchMoreY(double angle, double hypotenuse), HowMuchMoreX(double angle,
		double hypotenuse);
int calcDistancePassed();
double speed1, speed2, sonicT1 = 0, sonicT2 = 0;
int rega = 0; // chek

int main(void) {
	ClockSetup();
	InitGPIO();
	//InitUARTs();
	InitPIT();
	adc_init();
	RGB_LED_OFF;
	InitTPM(1);
	InitTPM(2);
	EncoderConfig();
	MotorConfig();
	//UARTprintf(UART0_BASE_PTR,
	//	"\r\n1. DC MOTORS and ENCODERS\r\n2. SERVO MOTORS\r\n3. IR Distance Measuring Sensors\r\n4. Ultra-Sonic Distance Measuring Sensors\r\n5. Reflectance Sensor\r\n6. Battery\r\n7. DTMF\n\r");
	TPM0_SC |= TPM_SC_CMOD(1); //Starts the TPM0 counter
	TPM1_SC |= TPM_SC_CMOD(1); //Start the TPM1 counter
	TPM2_SC |= TPM_SC_CMOD(1); //Start the TPM2 counter
	StartMove();
	
	return 0;
}

//-----------------------------------------------------------------//
//         PIT - ISR = Interrupt Service Routine                   //
//-----------------------------------------------------------------//
void PIT_IRQHandler() {
	switch (casePIT) {
	case 30: { // go through  the door - HODOR	
		if (measPit == 0) {
			RGB_LED_ON;
			measPit++;
		} else {
			RGB_LED_OFF;
			measPit = 0;
		}
		measureIRLeft();
		distArr[0][c] = ADC0_RA ;
		measureIRRight();
		distArr[1][c] = ADC0_RA ;
		c++;
		if (c == 15)
			c = 0;
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 15: { // go through  the door - HODOR	
		if (rega < 1000)
			rega++;
		else {
			GPIOC_PDOR = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 14: { // go through  the door - HODOR

		casePIT = 15;
		GPIOC_PDOR = 0x0440;
		TPM2_C1V = MUDULO_REGISTER; //Left engine 
		TPM1_C0V = MUDULO_REGISTER; //Right engine
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}

	case 13: { // finding an opening
		if (rightIR == 0) {
			measureIRLeft();
			if (ADC0_RA <= 2600) {
				int j = 0;
				for (; j < 1640000; j++)
				;
				GPIOC_PDOR = 0;
				for (j=0; j < 480000; j++)
				;
				turnLeft();
				casePIT = 14;
			}
		}
		else {
			measureIRRight();
			if (ADC0_RA <= 2600) {
				int j = 0;
				for (; j < 1640000; j++)
				;
				GPIOC_PDOR = 0;
				for (j=0; j < 480000; j++)
				;
				turnRight();
				casePIT = 14;
			}
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 

		break;
	}
	case 12: { // turn left or right and forward.
		GPIOC_PDOR = 0x0440;
		TPM2_C1V = MUDULO_REGISTER;//Left engine 
		TPM1_C0V = MUDULO_REGISTER;//Right engine
		casePIT = 13;
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 11: { // moving towards the wall
		ADC0_SC2 &= ~56;//cancels compare function.
		measureIRLeft();
		if (ADC0_RA >= 2700) {
			GPIOC_PDOR = 0;
			servo1 = 90;
			TPM1_C1V = 1071;
			int j = 0;
			for (; j < 960000; j++)
			;
			tempMoveCounterY = tempMoveCounterY + calcDistancePassed();
			casePIT = 12;
			turnRight();
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
			break;
		}
		measureIRRight();
		if (ADC0_RA >= 2700) {
			GPIOC_PDOR = 0;
			servo2 = 90;
			TPM0_C4V = 974;
			int j = 0;
			for (; j < 960000; j++)
			;
			tempMoveCounterY = tempMoveCounterY + calcDistancePassed();
			casePIT = 12;
			turnLeft();
		}

		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 10: {

		servo1 = 90;
		TPM1_C1V = 1071;
		servo2 = 90;
		TPM0_C4V = 974;
		casePIT = 666; //1
		rightClick = 0;
		leftClick = 0;
		GPIOC_PDOR = 0x0440;
		TPM2_C1V = MUDULO_REGISTER;//Left engine full speed
		TPM1_C0V = MUDULO_REGISTER;//Right engine full speed

		enable_irq(INT_ADC0 - 16);
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 666: { //////////////////////////////////////////////////////// for 1

		if (rega < 15) rega++;
		else {
			tempMoveCounterY = tempMoveCounterY + calcDistancePassed();
			rightClick = 0;
			leftClick = 0;
			casePIT = 1;
			rega = 0;

		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 9: {
		toStop--;
		if (toStop <= 0) {
			GPIOC_PDOR = 0; // stop the car

			if (doubleTurn == 1) {
				casePIT = 10; // double turn done, continue searching
				doubleTurn = 0;
				if (didITurnRight == 0)
				turnRight();
				else
				turnLeft();

			} else {
				doubleTurn++;
				if (didITurnRight == 1) {

					casePIT = 663;
					servo1 = 90;
					TPM1_C1V = 1071;
					int j = 0;
					for (; j < 96000; j++)
					;
					turnLeft();
				} else {

					casePIT = 662;
					servo2 = 90;
					TPM0_C4V = 972;
					int j = 0;
					for (; j < 96000; j++)
					;
					turnRight();
				}
			}
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 8: {
		toStop++;
		if (rightIR == 0) {
			measureIRLeft();
			int tempADC = ADC0_RA;
			if (tempADC <= 2416 || toStop > 40) {
				casePIT = 9; // turning back accordingly
				GPIOC_PDOR = 0x00A0;// reverse
				TPM2_C1V = MUDULO_REGISTER;//Left engine full speed
				TPM1_C0V = MUDULO_REGISTER;//Right engine full speed
				PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
				break;
			}
		} else {
			measureIRRight();
			int tempADC = ADC0_RA;
			if (tempADC <= 2416 || toStop > 40) {
				casePIT = 9; // turning back accordingly
				GPIOC_PDOR = 0x00A0;// reverse
				TPM2_C1V = MUDULO_REGISTER;//Left engine full speed
				TPM1_C0V = MUDULO_REGISTER;//Right engine full speed
			}
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 7: { //if object is close, slow down.
		toStop++;
		measureIRLeft();
		if (ADC0_RA >= 2300 || toStop > 35) {
			TPM2_C1V =MUDULO_REGISTER; //Left engine 
			TPM1_C0V = MUDULO_REGISTER;//Right engine 
			casePIT = 8;
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag 
			break;
		}
		measureIRRight();
		if (ADC0_RA >= 2300 || toStop > 35) {
			TPM2_C1V = MUDULO_REGISTER; //Left engine
			TPM1_C0V = MUDULO_REGISTER;//Right engine
			casePIT = 8;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}
	case 6: { // go straight towards object.
		rightClick = 0;
		leftClick = 0;

		casePIT = 7;
		int j = 0;
		for (; j < 96000; j++)
		;
		GPIOC_PDOR = 0x0440;
		TPM2_C1V = MUDULO_REGISTER;//Left engine full speed
		TPM1_C0V = MUDULO_REGISTER;//Right engine full speed
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag 
		break;
	}
	case 5: { // react to found object on left side.
		measureIRLeft();
		finalValidCounter++;
		if (ADC0_RA >= irCompare - 225 && ADC0_RA <= irCompare + 225) { //checks given value with range of 250 up and down.
			validObs++;
		}
		else if(finalValidCounter >= validObs + 5) {
			casePIT = 666;
			enable_irq(INT_ADC0 - 16);
			doubleTurn = 0;
			servo1 = 90;
			servo2 = 90;
			TPM1_C1V = 1071;
			TPM0_C4V = 974;
			int j = 0;
			for (; j < 960000; j++)
			;
			GPIOC_PDOR = 0x0440;
			TPM2_C1V = MUDULO_REGISTER; //Left engine full speed
			TPM1_C0V = MUDULO_REGISTER;//Right engine full speed
			validObs = 0;
			finalValidCounter = 0;
		}
		if (validObs == 10) {
			didITurnRight = 0;
			casePIT = 6;
			validObs = 0;
			finalValidCounter = 0;
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			int j = 0;
			for (; j < 960000; j++)
			;
			servo1 = 0;
			servo2 = 180;
			tempMoveCounterY = tempMoveCounterY+ calcDistancePassed();
			turnLeft();
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}
	case 665: { ////////////////////////////////////////////////////////for 5

		if (rega <10) rega++;
		else {
			casePIT = 5;
			rega = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}

	case 4: { // react to found object on right side.
		measureIRRight();
		finalValidCounter++;
		if (ADC0_RA >= irCompare - 225 && ADC0_RA <= irCompare + 225) {
			validObs++;
		}

		else if(finalValidCounter >= validObs + 5) {
			enable_irq(INT_ADC0 - 16);
			casePIT = 666;
			doubleTurn = 0;
			servo1 = 90;
			servo2 = 90;
			TPM1_C1V = 1071;
			TPM0_C4V = 974;
			int j = 0;
			for (; j < 960000; j++)
			;
			GPIOC_PDOR = 0x0440;
			TPM2_C1V = MUDULO_REGISTER; //Left engine full speed
			TPM1_C0V = MUDULO_REGISTER;//Right engine full speed
			validObs = 0;
			finalValidCounter = 0;
		}
		if (validObs == 10) {
			didITurnRight = 1;
			casePIT = 6;
			validObs = 0;
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			servo1 = 0;
			servo2 = 180;
			int j = 0;
			for (; j < 960000; j++)
			;
			finalValidCounter = 0;
			turnRight();
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}
	case 664: { ////////////////////////////////////////////////////////for 4

		if (rega < 10) rega++;
		else {
			casePIT = 4;
			rega = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 3: { //verify left servo obstacle.

		if (isValid1 == 1 && (sonicT1 <= 25)) {
			validObs++;
		}
		if (isValid1 == 1)
		finalValidCounter++;
		if (finalValidCounter >= validObs + 4) {
			finalValidCounter = 0;
			validObs = 0;
			casePIT = 665; // move to IR test left
			disable_irq(INT_ADC0 - 16);
		} else if (validObs == 10) {
			disable_irq(INT_ADC0 - 16);
			sonicToggle = 0;
			finalValidCounter = 0;
			validObs = 0; //clears counters.
			didITurnRight = 0;
			casePIT = 6;
			tempMoveCounterY =tempMoveCounterY+ calcDistancePassed();
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			int j = 0;
			for (; j < 960000; j++)
			;
			servo1 = 0;
			servo2 = 180;
			turnLeft();
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag
			break;
		}
		sonicToggle = 0;
		sonicMeasure();
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}
	case 663: { ////////////////////////////////////////////////////////for 3

		if (rega < 10) rega++;
		else {
			casePIT = 3;
			rega = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 2: { //reassure right

		if (isValid2 == 1 && (sonicT2 <= 25)) {
			validObs++;
		}
		if (isValid2 == 1)
		finalValidCounter++;
		if (finalValidCounter >= validObs + 4) {
			finalValidCounter = 0;
			validObs = 0;
			casePIT = 664; // move to case of IR
			disable_irq(INT_ADC0 - 16);
		} else if (validObs == 10) {
			casePIT = 6;
			tempMoveCounterY =tempMoveCounterY + calcDistancePassed();
			disable_irq(INT_ADC0 - 16);
			sonicToggle = 0;
			finalValidCounter = 0;
			validObs = 0; //clear counters.
			didITurnRight = 1;
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			int j = 0;
			for (j = 0; j < 960000; j++)
			;
			servo1 = 0;
			servo2 = 180;
			turnRight();
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag
			break;
		}
		sonicToggle = 1; //only right sensor.
		sonicMeasure();
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag 
		break;
	}
	case 662: { ////////////////////////////////////////////////////////for 2

		if (rega < 10) rega++;
		else {
			casePIT = 2;
			rega = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 1: { // open search

		if ((calcDistancePassed() + tempMoveCounterY) >= 550) { // done
			casePIT = 11;// change to exit case
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			servo1 = 0;
			servo2 = 180;
			disable_irq(INT_ADC0 - 16);
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag 
			break;
		}
		measureIRRight();
		casePIT = -2;
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}

	case -2: { // open search

		if ((calcDistancePassed() + tempMoveCounterY) >= 550) { // done
			casePIT = 11;
			TPM1_C1V = 455;
			TPM0_C4V = 1640;
			servo1 = 0;
			servo2 = 180;
			disable_irq(INT_ADC0 - 16);
			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag 
			break;
		}
		measureIRLeft();
		casePIT = 1;
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}
	case 661: { ////////////////////////////////////////////////////////for 2

		if (rega < 40) rega++;
		else {
			casePIT = 1;
			rega = 0;
		}
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
		break;
	}
	case 0: //START
	{
		if (((isValid1 == 1 && sonicT1 <= 60)
						&& (isValid2 == 1 && sonicT2 <= 60))) {
			DistSensConfig();
			casePIT = 661;
			isValid1 = 0;
			isValid2 = 0;
			rightClick = 0;
			leftClick = 0;
			tempMoveCounter = 0;
			tempMoveCounterY = 0;
			servo1 = 90;
			servo2 = 90;
			TPM1_C1V = 1071;
			TPM0_C4V = 974;
			GPIOC_PDOR = 0x0440;
			TPM2_C1V = MUDULO_REGISTER; //Left engine full speed
			TPM1_C0V = MUDULO_REGISTER;//Right engine full speed

			PIT_TFLG0 |= PIT_TFLG_TIF_MASK;//Turn off the Pit 0 Irq flag
			break;
		}

		sonicMeasure();
		PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Turn off the Pit 0 Irq flag 
		break;
	}

}
}
//=====================================================================================//
//                             ADC 0 WALLAK                                            //
//=====================================================================================//
void ADC0_IRQHandler() {
	if ((ADC0_SC2 & 56) != 0) {
		irCompare = ADC0_RA ;
	}
	int j = 0;
	for (; j < 296000; j++)
	;
	GPIOC_PDOR = 0;
	tempMoveCounterY =tempMoveCounterY + calcDistancePassed();
	rightClick = 0;
	leftClick = 0;
	if (rightIR == 0) {

		casePIT = 663;
	} else {

		casePIT = 662;
	}

	sonicMeasure();
}
//=====================================================================================//
//                             TPM 0 WALLAK                                            //
//=====================================================================================//
void FTM0_IRQHandler() {
	if (TPM0_STATUS & 0x08) { //Channel 3
		if (TPM0_MOD == MUDULO_REGISTER2) {
			if (sonicState2 == 0) {
				sonicT2 = TPM0_C3V ;
				sonicState2++;
				sonicTOF2 = 0;
				TPM0_STATUS |= 0x08;
			}
			else {
				sonicT2 = TPM0_C3V - sonicT2;
				if(sonicT2 < 0)
				sonicT2 = sonicT2 + MUDULO_REGISTER2;
				sonicState2 = 0;
				sonicT2 = sonicT2*0.023;
				if(sonicT2 >= 4 && sonicT2 <= 200) {
					isValid2 = 1;
				}
				else
				isValid2 = 0;

				SetBackToEncoders();

				TPM0_STATUS |= 0x08;
			}
		}
		else {
			OldTime2 = NewTime2 - TOF2old * MUDULO_REGISTER;
			NewTime2 = TOF2 * MUDULO_REGISTER + TPM0_C3V;
			TOF2old = TOF2;
			TOF2 = 0;
			leftClick++;
			speed2 = EncoderSensing(2);
			if(leftClick >= 296 && toTurn == 1) { //turn
							leftClick = 0;
							GPIOC_PDOR &= ~0x0480;
							if(GPIOC_PDOR == 0) { // 2nd to turn off.
								PIT_MCR &= ~PIT_MCR_MDIS_MASK;
								toTurn = 0;
								int j = 0;
								for (; j < 2400000; j++)
								;
							}
						}

					}
					TPM0_STATUS |= 0x08;
				}
				else if (TPM0_STATUS & 0x04) { //Channel 2
					if (TPM0_MOD == MUDULO_REGISTER2) {
						if (sonicState1 == 0) {
							sonicT1 = TPM0_C2V; //left
							sonicState1++;
							sonicTOF1 = 0;
							TPM0_STATUS |= 0x04;
						}
						else {
							sonicT1 = TPM0_C2V - sonicT1;
							if(sonicT1 < 0) {
								sonicT1 = sonicT1 + MUDULO_REGISTER2;
							}
							sonicState1 = 0;
							sonicT1 = sonicT1*0.023;
							if(sonicT1 >= 4 && sonicT1 <= 200) {
								isValid1 = 1;
							}
							else
							isValid1 = 0;

							SetBackToEncoders();

						}
						TPM0_STATUS |= 0x04;
					}
					else {
						OldTime1 = NewTime1 - TOF1old * MUDULO_REGISTER;
						NewTime1 = TOF1 * MUDULO_REGISTER + TPM0_C2V;
						TOF1old = TOF1;
						TOF1 = 0;
						rightClick++;
						speed1 = EncoderSensing(1);
						if(rightClick >= 296 && (toTurn == 1)) {
							rightClick = 0;
							GPIOC_PDOR &= ~0x0060;
							if(GPIOC_PDOR == 0) {
								PIT_MCR &= ~PIT_MCR_MDIS_MASK;
								toTurn = 0;
								int j = 0;
								for (; j < 2400000; j++)
								;
							}
						}
					}
					TPM0_STATUS |= 0x04;
				}
				else if ((TPM0_STATUS & 0x0100) || (TPM0_SC & 0x0080)) {
					TPM0_STATUS |= 0x0100;
					TPM0_SC |= 0x0080;
					TOF1++;
					TOF2++;
					//sonicTOF1++;
					//sonicTOF2++;

				}
				adjustCounter++;
				if (adjustCounter == 150) {
					AdjustSpeed();
					adjustCounter = 0;
				}
			}

//-----------------------------------------------------------------
// EncoderSensing
//-----------------------------------------------------------------
double EncoderSensing(int engine) {
	if (engine == 1) {
		if (TPM2_C1V != 0) {
			if (OldTime1 != 0 && NewTime1 != 0) {
				return( 34650 / (NewTime1 - OldTime1));

			} else {

				TOF1old = 0;

				OldTime1 = 0;
				return 0;
			}}
		else {

			TOF1old = 0;
			NewTime1 = 0;
			OldTime1 = 0;
			return 0;
		}}
	else {
		if (TPM1_C0V != 0) {
			if (OldTime2 != 0 && NewTime2 != 0) {
				return( 34650 / (NewTime2 - OldTime2));

			} else {

				TOF2old = 0;

				OldTime2 = 0;
				return 0;
			}}
		else {

			TOF2old = 0;
			NewTime2 = 0;
			OldTime2 = 0;
			return 0;
		}
	}
}
void AdjustSpeed() {
	if (speed1 > speed2) {
		double tempQ = (speed1 - speed2) * 164;
		TPM2_C1V = TPM2_C1V-tempQ;
		TPM1_C0V = TPM1_C0V+tempQ;
	}
	else if (speed2 > speed1) {
		double tempQ = (speed2 - speed1) * 164;
		TPM2_C1V = TPM2_C1V+tempQ;
		TPM1_C0V = TPM1_C0V-tempQ;
	}

}

