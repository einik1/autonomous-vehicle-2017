#include "TFC.h"
extern double sonicT1, sonicT2;
extern short casePIT, toTurn;
short isValid1 = 0, isValid2 = 0, adjDeg = 0;
extern int leftClick, rightClick, tempMoveCounter;
#define MUDULO_REGISTER 18750;
#define STEP 1875;
void StartMove() {
	ServosConfig();

	PIT_LDVAL0 = 0x249F00; //100 ms
	PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module

	int j = 0;
	for (; j < 9600000; j++)
		;
}

void testFunc() {
	ServosConfig();
	PIT_LDVAL0 = 0x16E3600; //100 ms
	casePIT = 30;
	tempMoveCounter = 60;
	PIT_MCR &= ~PIT_MCR_MDIS_MASK; //Enable the PIT module
}
int calcDistancePassed() {
	double avg = (rightClick + leftClick) * 0.5;
	avg = avg * 0.064; //0.064
	return (int) avg;
}
void turnLeft() {
	PIT_MCR |= PIT_MCR_MDIS_MASK;
	rightClick = 0;
	leftClick = 0;
	toTurn = 1;
	GPIOC_PDOR = 0x00C0;         //turn left.
	TPM2_C1V = 3*STEP;          //Right engine 
	TPM1_C0V = 3*STEP;          //Left engine 
}
void turnRight() {
	PIT_MCR |= PIT_MCR_MDIS_MASK;
	rightClick = 0;
	leftClick = 0;
	toTurn = 1;
	GPIOC_PDOR = 0x0420;         //turn right.
	TPM2_C1V = 3*STEP;          //Right engine 
	TPM1_C0V = 3*STEP;          //Left engine 
}
