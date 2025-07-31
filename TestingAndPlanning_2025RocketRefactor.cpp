/* planning out flight computer

test 1 is a basic test, collect data, tie flight states to time elapsed and dump ram buffer to Serial 

test 2 is a re run of test1 but with the added step of dumping the ram buffer to flash memory and then dumping the flash memory to Serial

test 3 
-----------------------------------------------------------------------------



using adadfruit icm20948 library
using adafruit bmp280 library
test different data rates 10, 20, 50, 100
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
const int ButtonPin

//object inits
Adafruit_BMP280 bmp;
Adafruit ICM_20948 icm;

//timing variables
unisgned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL_MS = 10; //10ms=100hz, 20ms=50hz, 50ms=10hz
unisgned long liftoffTime = -1;

//flight data, init struct, set initial values
struct __attribute__ ((packed)) FlightData {
	unisgned long timestamp;
	float temperature;
	float pressure;
	float alttude;
	float accelX, accelY, accelZ;
	float gyroX, gyroY, gyroZ;
	uint8_t flight_state
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


void setup(){
 Serial.begin(115200);
 while(!Serial && millis() < 5000);
 
 //setup pins
 
 Wire.begin();
 Wire.setClock(400000);
 
 //init bmp
 //init icm
 
 //set ranges for bmp
 //set ranges for icm
 //set rates for bmp
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
	//print csv header
	
	for(sizt_t i = 0; i < ramBufferIndex; i += sizeof(FlightData)){
		FlightData retrievedData;
		memcpy(&retrievedData, &ramBuffer[i]; sizeof(FlightData));
		char lineBuffer[256];
		snprintf(lineBuffer, sizeof(lineBuffer), "", SEE PACKET FOR REST OF THIS LINE);
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