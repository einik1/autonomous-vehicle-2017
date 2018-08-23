#include "TFC.h"
#define ADC_MAX_CODE    4095 // For 3.3[v]
#define MUDULO_REGISTER  0x493E // 18,750
#define MUDULO_REGISTER2 50000 //
#define STEP MUDULO_REGISTER/10 // 1,875
#define CM10 121290
#define CM15 124455
#define CM20 124908
#define CM125 800
#define CM130 500
int prevLoc = -1, i = 0;
extern short casePIT, sonicToggle, rightIR;
extern int distArr[2][15], notCalib;
extern uint8_t sonicTOF1, sonicTOF2, sonicState1, sonicState2;
extern short irCompare;

void DistSensConfig() { //for actual measuring - NOT CALIBRATING. 
	ADC0_SC3 |= ADC_SC3_AVGE_MASK + ADC_SC3_AVGS(3); // Set HW average - 32 samples.
	ADC0_CV1 = CM125; //Lower limit of compare.
	ADC0_CV2 = 3100; //Upper limit of compare.
	ADC0_SC2 |= 56; //Enables compare function, set to values between CV1>= and <=CV2
}

void SetBackToEncoders() {
	TPM0_SC = 0; // to ensure that the counter is not running
	TPM0_SC |= TPM_SC_PS(5) + TPM_SC_TOIE_MASK; //Prescaler = 32, up-mode, counter-disable
	TPM0_C2SC = 0;
	TPM0_C2SC |= TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK;
	TPM0_C3SC = 0;
	TPM0_C3SC |= TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK;
	TPM0_MOD = MUDULO_REGISTER;
	PORTC_PCR3 = PORT_PCR_MUX(4); //– TPM0_CH2 
	PORTC_PCR4 = PORT_PCR_MUX(4); //– TPM0_CH3
	PORTE_PCR31 = PORT_PCR_MUX(3); //– TPM0_CH4
	TPM0_C1SC = 0;
	TPM0_STATUS |= 0x0C;
	PORTD_PCR1 = PORT_PCR_MUX(0); // default
	PORTD_PCR2 = PORT_PCR_MUX(0); // default
	PORTD_PCR3 = PORT_PCR_MUX(0); // default
	TPM0_SC |= TPM_SC_CMOD(1); // start the clock

	PIT_MCR &= ~PIT_MCR_MDIS_MASK;

}
void measureIRLeft() {
	ADC0_SC1A = ADC_SC1_ADCH(7) + ADC_SC1_AIEN_MASK; //Left Sensor.
	rightIR = 0;
	int j = 0;
	for (; j < 165295; j++)
	;

}

void measureIRRight() {

	rightIR = 1;
	ADC0_SC1A = ADC_SC1_ADCH(6) + ADC_SC1_AIEN_MASK; //Right Sensor.
	int j = 0;
	for (j = 0; j < 165295; j++)
		;

}

void sonicMeasure() {

	PIT_MCR |= PIT_MCR_MDIS_MASK;

	TPM0_SC = 0;
	TPM0_SC |= TPM_SC_PS(5); // clock divide by 32 + over flow enable //changed
	TPM0_MOD = MUDULO_REGISTER2; // freq = 15Hz

	TPM0_C2SC = 0;
	TPM0_C3SC = 0;

	PORTC_PCR3 = PORT_PCR_MUX(0); //– TPM0_CH2 
	PORTC_PCR4 = PORT_PCR_MUX(0); //– TPM0_CH3
	PORTE_PCR31 = PORT_PCR_MUX(0); //turns off configuration to one of the servo motors.

	PORTD_PCR1 = PORT_PCR_MUX(4); // TPM0_CH1
	PORTD_PCR2 = PORT_PCR_MUX(4); // TPM0_CH2
	PORTD_PCR3 = PORT_PCR_MUX(4); // TPM0_CH3

	if (sonicToggle == 0) {
		sonicToggle = 1;
		TPM0_C2SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK; //Capture on rising and falling

	} else {

		sonicToggle = 0;
		TPM0_C3SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK; //Capture on rising and falling
	}
	TPM0_C1SC = 0;
	TPM0_C1SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_MSB_MASK;  // PWM
	TPM0_C1V = 8;  // 12 micro-sec 
	TPM0_SC |= TPM_SC_CMOD(1);  // start clock
	sonicTOF1 = 0;
	sonicState1 = 0;
	sonicState2 = 0;

}
