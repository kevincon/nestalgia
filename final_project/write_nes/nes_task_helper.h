#ifndef _NES_TASK_HELPER_H_
#define _NES_TASK_HELPER_H_

const uint8_t NIN_A = 0;
const uint8_t NIN_B = 1;
const uint8_t NIN_SELECT = 2;
const uint8_t NIN_START = 3;
const uint8_t NIN_UP = 4;
const uint8_t NIN_DOWN = 5;
const uint8_t NIN_LEFT = 6;
const uint8_t NIN_RIGHT = 7;

void process_input(char c, uint8_t *buttons) {
	uint8_t i;
	
	for(i = 0; i < 8; i++)
		buttons[i] = 1;
	
	switch(c) {
		case 0x4B: // K
		case 0x6B: // k
			buttons[NIN_A] = 0;
			break;
		case 0x4A: // J
		case 0x6A: // j
			buttons[NIN_B] = 0;
			break;
		case 0x9: // TAB
			buttons[NIN_SELECT] = 0;
			break;
		case 0xD: // ENTER
			buttons[NIN_START] = 0;
			break;
		case 0x57: // W
		case 0x77: // w
			buttons[NIN_UP] = 0;
			break;
		case 0x53: // S
		case 0x73: // s
			buttons[NIN_DOWN] = 0;
			break;
		case 0x41: // A
		case 0x61: // a
			buttons[NIN_LEFT] = 0;
			break;
		case 0x44: // D
		case 0x64: // d
			buttons[NIN_RIGHT] = 0;
			break;
	}
	
	/*printf("You pressed %c, which has a code of %x\r\n", c, c);
	
	nrk_kprintf( PSTR("BUTTONS: " ));
	for(i = 0; i < 8; i++) {
		printf("%d ", buttons[i]);
	}
	nrk_kprintf( PSTR("\r\n" ));
	*/
}

#endif
