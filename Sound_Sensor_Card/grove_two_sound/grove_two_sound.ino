
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>


#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define SOUND_LED_PIN_NUM		PA1

#define SOUND_POWER_PIN_NUM     PA4

#define SOUND_INPUT_PIN_NUM		PA0

/***************************************************************

 ***************************************************************/
#define SOUND_I2C_ADDRESS		0x06
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0006

#define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address
#define I2C_THD_0_ADDR_FLASH_LOC	0x01 // int address
#define I2C_THD_1_ADDR_FLASH_LOC	0x02 // int address
#define I2C_THD_2_ADDR_FLASH_LOC	0x03 // int address

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 // 
#define I2C_CMD_GET_SOUND		0x02 // 
#define I2C_CMD_SET_THD			0x03 // 
#define I2C_CMD_LED_ON			0xb0 // 
#define I2C_CMD_LED_OFF			0xb1 // 
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 // 
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 // 
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 // 
#define I2C_CMD_TEST_TX_RX_ON   0xe0 // 
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 // 
#define I2C_CMD_TEST_GET_VER    0xe2 // 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 // 
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff //

uint16_t deviceI2CAddress =  SOUND_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;

typedef struct
{
	uint16_t deviceVID;
	uint16_t devicePID;
	uint32_t deviceEvent;
}packet_01_t; // 8 bytes

union
{
    packet_01_t data;
    uint8_t bytes[8]; 
}packet_01_data;

uint8_t *ptr1 = (uint8_t *)&packet_01_data;

void requestEvent(void);
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT	2000

uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME	250

bool ledFlashCommand = false;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

/***************************************************************

 ***************************************************************/
#define SOUND_THD_0_NUM	50
#define SOUND_THD_1_NUM	200

uint16_t soundThd0 = SOUND_THD_0_NUM;
uint16_t soundThd1 = SOUND_THD_1_NUM;
uint16_t curSoundData = 0;

#define SOUND_SAMPPLE_TIME  50

uint32_t samplePreviousMillis = 0;
uint16_t maxSoundData = 0;
uint16_t minSoundData = 1023;

bool testFlag = false;
char *versions = "V15";

uint32_t intStart = 0;
uint32_t intEnd = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
bool flashSave = false;

/***************************************************************

 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    
    uint8_t *ptr2 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr2 + i);
	
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, SOUND_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, SOUND_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	uint16_t soundThd = Flash.read16(I2C_THD_0_ADDR_FLASH_LOC);
	if(soundThd == 0xffff)Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, SOUND_THD_0_NUM);
	else soundThd0 = soundThd;
	
	soundThd = Flash.read16(I2C_THD_1_ADDR_FLASH_LOC);
	if(soundThd == 0xffff)Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, SOUND_THD_1_NUM);
	else soundThd1 = soundThd;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
	
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(SOUND_LED_PIN_NUM, OUTPUT);
	digitalWrite(SOUND_LED_PIN_NUM, HIGH);
    
    pinMode(SOUND_POWER_PIN_NUM, OUTPUT);
    digitalWrite(SOUND_POWER_PIN_NUM, HIGH);
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
	
	wwdg.begin();
    
    // Serial.begin(115200);
}

void loop()
{
	uint16_t sampleData = analogRead(SOUND_INPUT_PIN_NUM);
    if(maxSoundData < sampleData)maxSoundData = sampleData;
    else if(minSoundData > sampleData)minSoundData = sampleData;
    
    uint32_t sampleCurrentMillis = millis(); 
    if(sampleCurrentMillis - samplePreviousMillis >= SOUND_SAMPPLE_TIME)
    {
        samplePreviousMillis = sampleCurrentMillis;
        
        curSoundData = maxSoundData - minSoundData;
        maxSoundData = 0;
        minSoundData = 1023;
        
        // Serial.println(curSoundData);
    }
	
	if(curSoundData < soundThd0)packet_01_data.data.deviceEvent = 1;
	else if(curSoundData >= soundThd0 && curSoundData < soundThd1)packet_01_data.data.deviceEvent = 2;
	else if(curSoundData >= soundThd1)packet_01_data.data.deviceEvent = 3;
	
	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(SOUND_LED_PIN_NUM, ledFlashStatus);
			ledFlashStatus = !ledFlashStatus;
		}
	}
	
	if(commandReceive == I2C_CMD_SET_ADDR) // change i2c address
	{
		commandReceive = I2C_CMD_NULL;
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
	else if(commandReceive == I2C_CMD_RST_ADDR) // reset i2c address
	{
		commandReceive = I2C_CMD_NULL;
		deviceI2CAddress = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
	else if(commandReceive == I2C_CMD_SET_THD) // set new threshold
	{
		commandReceive = I2C_CMD_NULL;
        if(flashSave)
        {
            Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, soundThd0);
            Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, soundThd1);
        }		
	}
	
	if(autoSleepFlag)
	{
		uint32_t autoSleepCurrentMillis = millis();
		if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
		{
			autoSleepPreviousMillis = autoSleepCurrentMillis;
			
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(SOUND_LED_PIN_NUM, HIGH);
            
            digitalWrite(SOUND_POWER_PIN_NUM, LOW);
			
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
            digitalWrite(SOUND_POWER_PIN_NUM, HIGH);
            
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
		}
	}

    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TWO_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_TWO_RX_PIN_NUM, OUTPUT);
        
        while(1)
        {           
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            if(testFlag == false)break;
        }
        
        wwdg.begin();
        attachInterrupt(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);	
    }
    
	wwdg.reset();
}

void dummy(void)
{
	autoSleepPreviousMillis = millis();
    
    if(digitalRead(GROVE_TWO_RX_PIN_NUM) == LOW)intStart = millis();
    else 
    {
        intEnd = millis();
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void receiveEvent(int howMany)
{
	uint8_t i = 0, receiveBuffer[8] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{	
		receiveBuffer[i ++] = Wire.read();
		if(i >= 8)i = 0;
	}
	
	commandReceive = receiveBuffer[0];
	
	switch(commandReceive)
	{
		case I2C_CMD_SET_THD:
			{
				uint16_t soundThd = (int16_t)receiveBuffer[2] + receiveBuffer[3] * 256;
				if(receiveBuffer[1] == 0)soundThd0 = soundThd;
				else if(receiveBuffer[1] == 1)soundThd1 = soundThd;
                flashSave = receiveBuffer[4];
			}
		break;
		
		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(SOUND_LED_PIN_NUM, HIGH);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_AUTO_SLEEP_ON:
			autoSleepFlag = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_AUTO_SLEEP_OFF:
			autoSleepFlag = false;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_SET_ADDR:
			deviceI2CAddress = receiveBuffer[1];
		break;
		
        case I2C_CMD_TEST_TX_RX_ON:
            testFlag = true;
			commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_TEST_TX_RX_OFF:
            testFlag = false;
			commandReceive = I2C_CMD_NULL;
        break;
        
		case I2C_CMD_JUMP_TO_BOOT:
			commandReceive = I2C_CMD_NULL;
			jumpToBootloader();
		break;
		
		default:
		break;
	}
}

void requestEvent(void)
{
	// autoSleepPreviousMillis = millis();
	
	switch(commandReceive)
	{
		case I2C_CMD_GET_DEV_ID:
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_GET_DEV_EVENT:
			Wire.write(ptr1 + 4, 4);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_GET_SOUND:
			Wire.write((uint8_t)(curSoundData & 0xff));
			Wire.write((uint8_t)(curSoundData >> 8));
			commandReceive = I2C_CMD_NULL;
		break;
        
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_DEVICE_UID:
            Wire.write(chipId, 12);
            commandReceive = I2C_CMD_NULL;
        break;
		
		default:
		break;
	}
}
