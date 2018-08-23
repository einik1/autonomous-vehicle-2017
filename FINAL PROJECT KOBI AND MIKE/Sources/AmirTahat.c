#include "TFC.h"

void toPrint(int printVar) {
	int temp = (int) printVar, temp2 = 0, temp3 = 0;
	char str[2];
	str[1] = 0;
	if (printVar >= 1000) {
		temp = printVar / 1000;
		str[0] = ((int) temp + 48);
		UARTprintf(UART0_BASE_PTR, str);
		temp3 = temp;
		temp = printVar - 1000 * temp;

	} else {
		UARTprintf(UART0_BASE_PTR, "0");
	}
	if (printVar >= 100) {
		temp = temp / 100;
		str[0] = ((int) temp + 48);
		UARTprintf(UART0_BASE_PTR, str);
		temp2 = temp;
		temp = printVar - 100 * temp - 1000 * temp3;

	} else {
		UARTprintf(UART0_BASE_PTR, "0");
	}
	if (printVar >= 10) {
		temp = temp / 10;
		str[0] = ((int) temp + 48);
		UARTprintf(UART0_BASE_PTR, str);
		temp = printVar - 10 * temp - temp2 * 100 - 1000 * temp3;
	} else {
		UARTprintf(UART0_BASE_PTR, "0");
	}
	str[0] = ((int) temp + 48);
	UARTprintf(UART0_BASE_PTR, str);
	UARTprintf(UART0_BASE_PTR, "[cm]");
}
