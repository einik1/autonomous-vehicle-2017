#include "TFC.h"
#define ADC_MAX_CODE    4095 // For 3.3[v]
#define MUDULO_REGISTER  0x493E // 18,750
#define STEP MUDULO_REGISTER/10 // 1,875
extern int servo1, servo2;
extern short servoDir1,servoDir2;
void ServosConfig() {
	TPM0_SC = 0; // to ensure that the counter is not running
	TPM0_SC |= TPM_SC_PS(5); //Prescaler = 8, up-mode, counter-disable
	PORTE_PCR21 = PORT_PCR_MUX(3);  //– TPM1_CH1 
	PORTE_PCR31 = PORT_PCR_MUX(3);  //– TPM0_CH4
	TPM1_C1SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK;
	TPM1_C1V = 455;//763
	TPM0_C4SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK;
	TPM0_C4V = 1640;//1289
	TPM0_MOD = MUDULO_REGISTER;
	TPM0_SC |= TPM_SC_CMOD(1);
	servo1 = 45;
	servo2 = 135;
	servoDir1 = 1;
	servoDir2 = -1;
}

void Servo1SetPos(int Servo1Position) {//right
	TPM1_C1V = 455+6.85*Servo1Position;
	servo1 = Servo1Position;
}

void Servo2SetPos(int Servo2Position) {//left
	TPM0_C4V = 344+7*Servo2Position;
	servo2 = Servo2Position;
}
