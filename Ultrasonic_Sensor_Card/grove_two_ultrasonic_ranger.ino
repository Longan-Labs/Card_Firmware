
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer3.h>


#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define ULTRASONIC_BOOT_EN_PIN      PB1
#define ULTRASONIC_PWM_OUT_PIN      PA4
#define ULTRASONIC_TX_EN_PIN        PA0
#define ULTRASONIC_SIGNAL_OUT_PIN   PA6
#define ULTRASONIC_WAVE_RAW_PIN     PA7
#define ULTRASONIC_POWER_PIN        PA5

uint16_t ultraTimeCount = 0;
bool RXcomplete = false;
bool RXcount = false;
uint16_t waveCountTimer[256];
uint8_t waveNum = 0;

bool thldFlag = false;
uint32_t timeoutPreviousMillis = 0;

uint32_t curDistData = 0, distancePre = 0;
bool distanceFlag = false;

uint32_t powerVoltage = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x23
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000f

#define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address
#define I2C_THD_0_ADDR_FLASH_LOC	0x01 // int address
#define I2C_THD_1_ADDR_FLASH_LOC	0x02 // int address

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_GET_DISTANCE    0x02 // 
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
#define I2C_CMD_GET_VOLT        0xe3 // 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 // 
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff // 

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
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

void requestEvent();
void receiveEvent(int howMany);

/***************************************************************
Basic defines
 ***************************************************************/
LowPower nrgSave;

#define CLICK_CHECK_TIMEOUT	1000
#define AUTO_SLEEP_TIMEOUT	2000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

/***************************************************************

 ***************************************************************/
#define DISTANCE_THD_0_NUM	100
#define DISTANCE_THD_1_NUM	300

uint16_t distThd0 = DISTANCE_THD_0_NUM;
uint16_t distThd1 = DISTANCE_THD_1_NUM;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME	250

bool ledFlashCommand = false;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V10";

uint32_t intStart = 0;
uint32_t intEnd = 0;

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
bool flashSave = false;

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{
    uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
    uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);

    uint8_t *ptr2 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr2 + i);

    if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    else deviceI2CAddress = i2cCurrentAddr;

    uint16_t distThd = Flash.read16(I2C_THD_0_ADDR_FLASH_LOC);
    if(distThd == 0xffff)Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, DISTANCE_THD_0_NUM);
    else distThd0 = distThd;

    distThd = Flash.read16(I2C_THD_1_ADDR_FLASH_LOC);
    if(distThd == 0xffff)Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, DISTANCE_THD_1_NUM);
    else distThd1 = distThd;

    packet_01_data.data.deviceVID = DEVICE_VID;
    packet_01_data.data.devicePID = DEVICE_PID;
    packet_01_data.data.deviceEvent = 0;

    nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);

    pinMode(ULTRASONIC_BOOT_EN_PIN, OUTPUT);
    pinMode(ULTRASONIC_TX_EN_PIN, OUTPUT);
    pinMode(ULTRASONIC_WAVE_RAW_PIN, INPUT);

    digitalWrite(ULTRASONIC_BOOT_EN_PIN, HIGH);
    digitalWrite(ULTRASONIC_TX_EN_PIN, LOW);
    analogWrite(ULTRASONIC_PWM_OUT_PIN, 0);

    attachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN, waveCount, FALLING, INPUT_PULLUP);

    Timer3.init(10); // 10us
    Timer3.attachInterrupt(timerIsr);
    TIM_Cmd(TIM3, DISABLE);

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
    
    // Serial.begin(115200);
}

void loop()
{
    ultrasonicMeasureStartType();
    if(curDistData < distThd0)packet_01_data.data.deviceEvent = 1;
    else if(curDistData >= distThd0 && curDistData < distThd1)packet_01_data.data.deviceEvent = 2;
    else if(curDistData >= distThd1)packet_01_data.data.deviceEvent = 3;

    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(ULTRASONIC_POWER_PIN) * 3300 * 2 / 1024;
    }
    
    if(ledFlashCommand)
    {
        uint32_t ledFlashCurrentMillis = millis();
        if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
        {
            ledFlashPreviousMillis = ledFlashCurrentMillis;
            digitalWrite(GROVE_LED_PIN_NUM, ledFlashStatus);
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
            Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, distThd0);
            Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, distThd1);
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
            digitalWrite(GROVE_LED_PIN_NUM, HIGH);
            
            digitalWrite(ULTRASONIC_BOOT_EN_PIN, LOW);
            digitalWrite(ULTRASONIC_TX_EN_PIN, LOW);
            analogWrite(ULTRASONIC_PWM_OUT_PIN, 0);
    
            pinMode(ULTRASONIC_SIGNAL_OUT_PIN, OUTPUT);
            digitalWrite(ULTRASONIC_SIGNAL_OUT_PIN, LOW);
            // detachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN);
            
            wwdg.end();
            Wire.end();
            pinMode(PA9, INPUT_PULLUP);
            pinMode(PA10, INPUT_PULLUP);

            nrgSave.standby(true);

            Wire.begin(deviceI2CAddress);
            Wire.onReceive(receiveEvent);
            Wire.onRequest(requestEvent);
            wwdg.begin();
            
            digitalWrite(ULTRASONIC_BOOT_EN_PIN, HIGH);
            attachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN, waveCount, FALLING, INPUT_PULLUP);
        }
    }

    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_RX_PIN_NUM, OUTPUT);

        while(1)
        {           
            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);

            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);

            if(testFlag == false)break;
        }

        wwdg.begin();
        attachInterrupt(GROVE_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);	
    }

    wwdg.reset();
}

void dummy(void)
{
    autoSleepPreviousMillis = millis();    

    if(digitalRead(GROVE_RX_PIN_NUM) == LOW)intStart = millis();
    else 
    {
        intEnd = millis();
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void receiveEvent(int howMany)
{
    uint8_t i = 0, receiveBuffer[4] = {0,};
    // autoSleepPreviousMillis = millis();

    while(Wire.available())
    {	
        receiveBuffer[i ++] = Wire.read();
        if(i >= 4)i = 0;
    }

    commandReceive = receiveBuffer[0];

    switch(commandReceive)
    {        
        case I2C_CMD_SET_THD:
        {
            int16_t distThd = (int16_t)receiveBuffer[2] + receiveBuffer[3] * 256;
            if(receiveBuffer[1] == 0)distThd0 = distThd;
            else if(receiveBuffer[1] == 1)distThd1 = distThd;
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
            digitalWrite(GROVE_LED_PIN_NUM, HIGH);
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
            packet_01_data.data.deviceEvent = 0;
        break;

        case I2C_CMD_GET_DISTANCE:
            Wire.write((uint8_t)((curDistData) & 0xff));
			Wire.write((uint8_t)((curDistData) >> 8));
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_VOLT:
            Wire.write(powerVoltage & 0xff);
            Wire.write((powerVoltage >> 8) & 0xff);
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

/***************************************************************
 Device driver
 ***************************************************************/
void ultrasonicMeasureStartType(void)
{
    uint32_t distance = 0;

    if(RXcount)
    {
        if((millis() - timeoutPreviousMillis) > 30)
        {
            RXcomplete = false;
            RXcount = false;

            TIM_Cmd(TIM3, DISABLE);

            if(waveNum)
            {
                if(thldFlag)distance = ((waveCountTimer[0] * 10 + 1970 + 300) * 172 + 5000); // unit: centimeter * 10000
                else distance = ((waveCountTimer[0] * 10) * 172 + 5000); // unit: centimeter * 10000
                
                if(distance < 100000)distance += 15000; // 
                else if(distance >= 100000 && distance < 150000)distance += 10000; // 
                else if(distance >= 150000 && distance < 200000)distance += 5000; // 
                else if(distance >= 200000 && distance < 250000)distance += 1000; // 
                else if((distanceFlag == false) && (distance > 400000) && (distance < 900000))
                {
                    distanceFlag = true;
                    thldFlag = true;
                    distancePre = distance;
                    return ;
                }
                else if(distanceFlag == true)
                {
                    distance = (distancePre + distance) / 2;
                }

                curDistData = distance / 1000; // uint: millimeter
                
                // Serial.println(curDistData);
                
                if(thldFlag)thldFlag = false;
                distanceFlag = false;
            }
            else 
            {
                if(thldFlag)
                {
                    thldFlag = false;
                    curDistData = 0;
                }
                else
                {
                    thldFlag = true;
                }
            }
        }
    }
    else
    {
        ultrasonicTxStart(127, thldFlag);

        if(thldFlag) // for low threshold
        {
            delayMicroseconds(250);
        }
        else // for high threshold
        {
            delayMicroseconds(150);
            ultrasonicRxStart();
            // delayMicroseconds(150);
        }

        digitalWrite(ULTRASONIC_TX_EN_PIN, LOW);  // disable TX circuit
        // analogWrite(ULTRASONIC_PWM_OUT_PIN, 255);   // desable PWM output
        TIM_SetCompare1(TIM14, 25);

        if(thldFlag) // skip diffract signal
        {
            // digitalWrite(GROVE_LED_PIN_NUM, LOW);
            delay(2); // real delay is 1.97 ms
            // digitalWrite(GROVE_LED_PIN_NUM, HIGH);
            ultrasonicRxStart();
        }
    }
}

void ultrasonicTxStart(uint8_t duty, bool threshold)
{
    if(threshold)
    {        
        pinMode(ULTRASONIC_WAVE_RAW_PIN, OUTPUT);
        digitalWrite(ULTRASONIC_WAVE_RAW_PIN, LOW);
        delayMicroseconds(500); // wait for low threshold voltage stabilized

        digitalWrite(ULTRASONIC_TX_EN_PIN, HIGH); //Enable TX circuit
        // analogWrite(ULTRASONIC_PWM_OUT_PIN, duty); //set to 40kHz output
        TIM_SetCompare1(TIM14, 13); //set to 40kHz output
    }
    else 
    {
        pinMode(ULTRASONIC_WAVE_RAW_PIN, INPUT);    
        delayMicroseconds(1500); // wait for high threshold voltage stabilized

        digitalWrite(ULTRASONIC_TX_EN_PIN, HIGH); //Enable TX circuit
        // analogWrite(ULTRASONIC_PWM_OUT_PIN, duty); //set to 40kHz output
        TIM_SetCompare1(TIM14, 13); //set to 40kHz output
    }
}

void ultrasonicRxStart(void)
{
    ultraTimeCount = 0;
    RXcomplete = false;
    RXcount = true;
    waveNum = 0;
    memset(waveCountTimer, 0, sizeof(waveCountTimer));

    // TIM_SetCounter(TIM3, 0); // clean the timer counter
    TIM3->CNT = 0; // clean the timer counter

    // TIM_Cmd(TIM3, ENABLE); // enable timer
    TIM3->CR1 |= TIM_CR1_CEN; // enable timer

    timeoutPreviousMillis = millis();
}

void timerIsr()
{
    ultraTimeCount ++;
}

void waveCount()
{
    if(waveNum < 256)waveCountTimer[waveNum ++] = ultraTimeCount;
}
