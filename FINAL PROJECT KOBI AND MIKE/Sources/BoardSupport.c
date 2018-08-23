#include "TFC.h"
#include "mcg.h"

#define MUDULO_REGISTER  0x493E // 18,750
// set I/O for switches and LEDs
void InitGPIO() {
	//enable Clocks to all ports - page 206, enable clock to Ports
	SIM_SCGC5 |=
			SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK
					| SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Red  
	PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green
	GPIOB_PDDR |= RED_LED_LOC + GREEN_LED_LOC; //Setup as output pins

	//Setup PortB pins.
	PORTB_PCR0 = PORT_PCR_MUX(1);
	PORTB_PCR1 = PORT_PCR_MUX(1);
	PORTB_PCR2 = PORT_PCR_MUX(1);
	PORTB_PCR3 = PORT_PCR_MUX(1);
	GPIOB_PDDR &= ~PORT_LOC(3) + ~PORT_LOC(2) + ~PORT_LOC(1) + ~PORT_LOC(0); // PTB(0-3) is Input.
	PORTB_PCR0 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK; //Dipswitch, needed??
	PORTB_PCR1 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK; //Dipswitch, needed??
	PORTB_PCR2 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK; //Dipswitch, needed??
	PORTB_PCR3 |= PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_PFE_MASK; //Dipswitch, needed??

}
//-----------------------------------------------------------------
// DipSwitch data reading
//-----------------------------------------------------------------
uint8_t TFC_GetDIP_Switch() {
	uint8_t DIP_Val = 0;

	DIP_Val = (GPIOC_PDIR >> 4) & 0xF;

	return DIP_Val;
}
//-----------------------------------------------------------------
// TPMx - Initialization
//-----------------------------------------------------------------
void InitTPM(char x) {  // x={0,1,2}
	switch (x) {
	case 0:
		TPM0_SC = 0; // to ensure that the counter is not running
		TPM0_SC |= TPM_SC_PS(5); //Prescaler = 32, up-mode, counter-disable
		TPM0_MOD = MUDULO_REGISTER; // PWM frequency of 40Hz = 24MHz/(600000)
		TPM0_C2SC |= TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK;
		TPM0_C3SC |= TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK;
		TPM0_C2V = 0x0;
		TPM0_C3V = 0x0;
		TPM0_CONF = 192; // Debug mode,
		enable_irq(INT_TPM0 - 16);// Enable Interrupts 
		set_irq_priority(INT_TPM0 - 16, 1);// Interrupt priority = 1
		break;
		case 1:
		TPM1_SC = 0;// to ensure that the counter is not running
		TPM1_SC |= TPM_SC_PS(5)+TPM_SC_TOIE_MASK;//Prescaler = 32, up-mode, counter-disable
		TPM1_MOD = MUDULO_REGISTER;// PWM frequency of 40Hz = 24MHz/(32x18750)
		TPM1_C0SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;
		TPM1_C0V = 0x0;
		TPM1_CONF = 192;
		break;
		case 2:
		TPM2_SC = 0;// to ensure that the counter is not running
		TPM2_SC |= TPM_SC_PS(5)+TPM_SC_TOIE_MASK;//Prescaler =32 up-mode, counter-disable
		TPM2_MOD = MUDULO_REGISTER;//PWM frequency of 40Hz = 24MHz/(32x18750)
		TPM2_C1SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;
		TPM2_C1V = 0x0;
		TPM2_CONF = 0;
		break;
	}
}
//-----------------------------------------------------------------
// TPMx - Clock Setup
//-----------------------------------------------------------------
void ClockSetup() {

	pll_init(8000000, LOW_POWER, CRYSTAL, 4, 24, MCGOUT); //Core Clock is now at 48MHz using the 8MHZ Crystal

	//Clock Setup for the TPM requires a couple steps.
	//1st,  set the clock mux
	//See Page 124 of f the KL25 Sub-Family Reference Manual
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2=24MHz (See Page 196 of the KL25 Sub-Family Reference Manual
	SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual
	//Enable the Clock to the TPM0 and PIT Modules
	//See Page 207 of f the KL25 Sub-Family Reference Manual
	SIM_SCGC6 |=
			SIM_SCGC6_TPM0_MASK + SIM_SCGC6_TPM2_MASK + SIM_SCGC6_TPM1_MASK;
	// TPM_clock = 24MHz , PIT_clock = 48MHz

}
//-----------------------------------------------------------------
// PIT - Initialisation
//-----------------------------------------------------------------
void InitPIT() {
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable the Clock to the PIT Modules
	// Timer 0

	PIT_LDVAL0 = 0x249F00; //100 ms
	PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //enable PIT0 and its interrupt
	PIT_MCR |= PIT_MCR_FRZ_MASK; // stop the pit when in debug mode
	enable_irq(INT_PIT - 16); //  
	set_irq_priority(INT_PIT - 16, 2);  // Interrupt priority = 2
}

//-----------------------------------------------------------------------
// Encoder Initialisation
//--------------------------------------------------------------------------
void EncoderConfig() {
	PORTC_PCR3 = PORT_PCR_MUX(4);  //– TPM0_CH2 
	PORTC_PCR4 = PORT_PCR_MUX(4);//– TPM0_CH3
	InitTPM(0);
}

//-----------------------------------------------------------------
// MotorConfig
//-----------------------------------------------------------------

void MotorConfig() {
	PORTC_PCR5 = PORT_PCR_MUX(1); //pin is IO
	PORTC_PCR6 = PORT_PCR_MUX(1);//pin is IO
	PORTC_PCR7 = PORT_PCR_MUX(1);//pin is IO
	PORTC_PCR10 = PORT_PCR_MUX(1);//pin is IO
	GPIOC_PDDR |= 0x04E0;// OUTPOT
	PORTE_PCR23 = PORT_PCR_MUX(3);
	PORTE_PCR20 = PORT_PCR_MUX(3);

}
//-----------------------------------------------------------------
//               INIT DAC
//-----------------------------------------------------------------
void InitDAC() {
//When the DAC is enabled and the buffer is not enabled, 
//the DAC module always converts the data in DAT0 to analog output voltage.
// pin PTE30 is by default (ALT0) configured as DAC0_OUT
//VDDA reference voltage (Use this option for the best ADC operation).
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK; //DAC0 Clock Gate Control
	DAC0_C0 |= DAC_C0_DACEN_MASK + DAC_C0_DACRFS_MASK + DAC_C0_DACTRGSEL_MASK
			+ DAC_C0_LPEN_MASK;

}

