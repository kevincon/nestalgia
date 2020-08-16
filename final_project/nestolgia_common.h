#define GROUP_NUMER 18
#define PAN_ID 0x1234
#define SERVER_ADDR 1337

#define NUM_NODES 3
#define GUN_ADDR 0x1000 // GUN
#define LEFT_LEG_ADDR 0x1001 // LEFT LEG
#define RIGHT_LEG_ADDR 0x1002 // RIGHT LEG
#define ADDR_START 0x1000 // Used to index data arrays

#define NEGATIVE_X 0x01
#define POSITIVE_X 0x02
#define NEGATIVE_Y 0x04
#define POSITIVE_Y 0x08
#define NEGATIVE_Z 0x10
#define POSITIVE_Z 0x20
#define TRIGGER_PRESSED 0x40

#define SET_THRESHOLD 0x1
#define RECALIBRATE 0x2

const uint8_t NIN_A = 0;
const uint8_t NIN_B = 1;
const uint8_t NIN_SELECT = 2;
const uint8_t NIN_START = 3;
const uint8_t NIN_UP = 4;
const uint8_t NIN_DOWN = 5;
const uint8_t NIN_LEFT = 6;
const uint8_t NIN_RIGHT = 7;

const uint8_t MOVE_THRESH = 5; // 3

const uint8_t MOVING_NONE = 0;
const uint8_t MOVING_LEFT = 1;
const uint8_t MOVING_RIGHT = 2;

//Make sure this is in both Master and Slave code:
typedef struct {
	uint16_t adxl_x, adxl_y, adxl_z; 
} adc_data_t;

typedef struct {
	int32_t v_x, v_y, v_z; 
} veloc_data_t;
