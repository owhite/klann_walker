#define errorHalt(msg) {Serial.println(F(msg)); SysCall::halt();}
#define SLIDE_PACKET_SIZE 8
#define MAX_RECORD_LENGTH 5000
#define SD_FILE_NAME_SIZE 20
#define SD_FILE_KEY 'Z'

#define JOY_SLAVE_ADDRESS 0x09
#define PACKET_LEN 12
#define OLED_RESET 4

// eyeball states
#define E_IDLE             1  // no movement
#define E_RUNNING          2  
#define E_START            3  



