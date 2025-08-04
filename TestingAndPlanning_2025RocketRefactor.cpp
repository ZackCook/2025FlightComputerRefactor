/* planning out flight computer

test 1 is a basic test, collect data, tie flight states to time elapsed and dump ram buffer to Serial 

test 2 is a re run of test1 but with the added step of dumping the ram buffer to flash memory and then dumping the flash memory to Serial

test 3 
-----------------------------------------------------------------------------
MISC NOTES


using adafruit icm20948 library
using adafruit bmp280 library
test different data rates 10, 20, 50, 100
--------------------------------------------------
EXPECTED BEHAVIORS IN EACH FLIGHT STATE
- HOW WILL STATUS LED BAHAVE IN EACH STATE?????????

STARTUP
- initialization scripts are running, variables being set, memory being allocated, calibrations conducted and objects declared

IDLE
- looking for command to set to ARMED

ARMED
- collecting data 
- looking for liftoff

BOOST
- collecting data
- looking for burnout

COAST
- collecting data
- looking for apogee conditions, velocity approaching zero

APOGEE
- collecting data
- looking for freefall conditions

FREEFALL
- collecting data
- looking for parachute deployment

DESCENT
- collecting data
- looking for touchdown

LANDED
- data collection ends @ set amount of time after touchdown is confirmed, captureding the accel spike cause by touchdown
- begin data transferred
- looking for successful transfer of data

SAFED
- data has been transferred

ERROR
- 
--------------------------------------------
IMPORTANT VARIABLES AND FUNCTIONS

NAMES SUBJECT TO CHANGES

ramLogBuffer (byte[])
 - block of memory holding FlightData samples
 - memcpy used to copy new samples into array
 - memcpy FLightData records out of this array when dumping values
 
ramLogIndex (size_t)
 - Current write pointer within RAM buffer, how many bytes are stored in the buffer
 - Tells how many samples have been written. where next sample will be written, shows how full buffer is
 - after each sample is added, this value shall be incremented by sizeof(FlightData)
 
MAX_RAM_LOG_SIZE
 - total capacity of RAM buffer 
 - how many bytes of RAM are dedicated to holding FLightData samples
 - Used when declaring size of RAM buffer and when determining if there is enough room for another sample
 
MAX_SAMPLES
 - max number of samples that can be held in buffer
 -  = MAX_RAM_LOG_SIZE/sizeof(FLightData)
 
currentSamples 
 - = ramLogIndex/sizeof(FLightData)
 
MAX_DURATION_SECONDS
 - (MAX_SAMPLES)/(SENSOR_READ_INTERVAL_MS) * 1000
 

-----------------------------------------------------------------------------

*/


#include Arduino
#include wire
#include adafruit_sensor
#include adafruit_BMP280
#include adafruit_ICM20948
#include adafruit_ICM20X

//function protos


//pin inits
const int buttonPin = -999;
const int statusLedPin = -999;

//object inits
Adafruit_BMP280 bmp;
Adafruit ICM_20948 icm;

//timing variables
unisgned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL_MS = 10; //10ms=100hz, 20ms=50hz, 50ms=10hz
unisgned long liftoffTime = -1;

//flight data, init struct, set initial values
struct __attribute__ ((packed)) FlightData {
	unisgned long timestamp; //4 bytes
	float temperature; // 4 bytes
	float pressure; // 4 bytes
	float altitude; //4 bytes
	float accelX, accelY, accelZ; //4+4+4=12 bytes
	float gyroX, gyroY, gyroZ; //4+4+4=12 bytes
	uint8_t flight_state // 1 byte
	//4+4+4+4+12+12+1=41 total bytes
}

//flight state enum
enum FlightState {
	STARTUP = 0,
	IDLE = 1,
	ARMED = 2,
	BOOST = 3,
	COAST = 4,
	APOGEE = 5,
	FREEFALL = 6,
	DESCENT = 7,
	LANDED = 8,
	SAFED = 9,
	ERROR = 999;
}
 FLightState currentFlightState = STARTUP;
 bool dataDumped = false; // keeps track of if ram buffer has been dumped, used to ensure data is dumped one time only

//ram buffer 
const size_t MAX_RAM_BUFFER_SIZE = X_bytes * 1024;
byte ramBuffer[MAX_RAM_BUFFER_SIZE]
size_t ramBufferIndex = 0;
int currentSamples = 0;


void setup(){
 Serial.begin(115200);
 while(!Serial && millis() < 5000);
 
 //setup pins
 pinMode(buttonPin, INPUT_PULLUP);
 pinMode(statusLedPin, OUTPUT)
 
 Wire.begin();
 Wire.setClock(400000);
 
 //init bmp
 if(!bmp.begin()){
	 Serial.println("Check bmp 280 wiring, a problem occured");
	 while(1);
 }
 //init icm
 
 //set ranges for bmp
 bmp.setSampling(ADAFRUIT_BMP280::MODE_NORMAL,
				ADAFRUIT_BMP280::SAMPLING_X2,
				ADAFRUIT_BMP280::SAMPLING_X8,
				ADAFRUIT_BMP280::FILTER_X4,
				ADAFRUIT_BMP280::STANDBY_MS_0_5);
 Serial.println("BMP parameters set");
 //set ranges for icm
 //set rates for icm
 
 
 currentFlightState = IDLE;
}

void loop(){
	handleButtonPress();
	
	unsigned long currentMillis = millis();
	
	if(currentFlightState >= ARMED && !dataDumped){
		if(currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS){
			lastSensorReadTime = currentMillis;
			logFlightData();
		}	
	}
	
	if(currentFlightState == IDLE){}
	if(currentFlightState == BOOST && time elapsed since transition >= 5){}
	if(currentFlightState == COAST && time elapsed since transition >= 8){}
	if(currentFlightState == APOGEE && time elapsed since transition >= 3){}
	if(currentFlightState == FREEFALL && time elapsd since transition >= 3){}
	if(currentFlightState == DESCENT && time elapsed since transition >= 12){}
	if(data has been succesfully transferred to correct storage medium){}
}

void dumpRamToSerial(){
	if(ramBufferIndex == 0){
		return;
	}
	
	Serial.println("Dumping data to serial");
	Serial.print("Current Samples: ");
	Serial.println(currentSamples);
	
	//print csv header
	Serial.println("Timestamp_ms, Temp_C, Pressure_Pa, ALT_m, ACCX_ms2, ACCY_ms2, ACCZ_ms2, GYROX_rads, GYROY_rads, GYROZ_rads, FlightState");
	
	for(sizt_t i = 0; i < ramBufferIndex; i += sizeof(FlightData)){
		FlightData retrievedData;
		memcpy(&retrievedData, &ramBuffer[i]; sizeof(FlightData));
		char lineBuffer[256];
		snprintf(lineBuffer, sizeof(lineBuffer), "%lu, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %u",
			retrievedData.timestamp,
			retrievedData.temperature,
			retrievedData.pressure,
			retrievedData.altitude,
			retrievedData.accelX,
			retrievedData.accelY,
			retrievedData.accelZ,
			retrievedData.gyroX,
			retrievedData.gyroY,
			retrievedData.gyroZ,
			(unsigned int)retrievedData.flight_state);
			
		Serial.println(lineBuffer);
		delay(1);
		Serial.println("Data dumped to serial")
	}
}

void logFlightData(){
	FlightData currentData;
	currentData.timestamp = millis(); //should this be millis or the currentMillis from the loop ???
	currentData.flight_state = currentFlightState;
	
	//bmp/imu events
	
	if(ramBufferIndex + sizeof(FlightData) <= MAX_RAM_BUFFER_SIZE){
		memcpy(&ramBuffer[ramBufferIndex], &currentData, sizeof(FlightData));
		ramBufferIndex += sizeof(FlightData)
		currentSamples = ramBufferIndex / sizeof(FlightData);
	}else{
		if(currentFlightState != ERROR){
			currentFlightState = ERROR;
			Serial.println("RAM Buffer full, ERROR);
		}
	}
	
}

void handleButtonPress(){
	static unisgned long lastButtonPressTime = 0;
	const unsigned long debounceDelay = 100;
	int buttonState = digitalRead(ButtonPin);
	if(buttonState == LOW && (millis()- lastButtonPressTime > debounceDelay)){
		switch(currentFlightState){
			case IDLE:
				currentFlightState = ARMED;
				break;
			case ARMED:
				currentFlightState = BOOST;
				liftoffTime = millis()
				break;
		}
	}
}

void setStatusLED(){
}