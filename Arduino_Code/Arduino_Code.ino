#include <DS3231.h>
#include <Wire.h>
#include <EEPROM.h>
#include <DHT.h>
#include <OneWire.h>
#include <DS18B20.h>

#define MED_FIL_ARR_SIZE 9    // Median filter window size. Must be odd
#define BAUDRATE 9600         // Baudrate for serial interface
#define TEMP_MEASR_DELAY_S 15 // For correct functioning, must be one of {2, 3, 4, 5, 6, 10, 12, 15, 20, 30}
#define LIGHT_INTENS_INCR 255 // Light intensity increment, for smooth transitioning (0 ~ 255)(the higher the valuee the less smooth the transition)

const byte humid_sensorPin = 10;
const byte heater_pin = 11;
const byte temp_outputPin = A2;
const byte temp_roomPin = 8;
const byte light_pin = 9;

// Real time clock (RTC) module DS3231
DS3231 Clock;

// Relative humidity and temperature sensor DHT22
DHT dht( humid_sensorPin, DHT22 );

// Temperature sensor DS18B20
OneWire oneWire(temp_roomPin);
DS18B20 ds18b20(&oneWire);

// Variables for communication interface and command processing
char cmd_buff[10], msg_byte;
byte cmd_i = 0;
bool dataSent = false;

// Variables for storing and filtering data from LM35 and DS18B20 temperature sensors
double temp_output, temp_room, temp_sortArr[MED_FIL_ARR_SIZE],
       temp_outputArr[MED_FIL_ARR_SIZE], temp_roomArr[MED_FIL_ARR_SIZE];

// Variables for storing data from DHT22 sensor
double temp_grow, humid_grow;

// Variables for PI temperature controller
double temp_ref, rtemp_target, rtemp_offset, rtemp_intg;
int temp_controlSig;

// Flags and counters for various functionalities
bool temp_initFinished = false, temp_measrUpdated = false, light_rampingUp = false, light_rampingDown = false, light_controlled = false, temp_controlled = false;
byte temp_initReads = MED_FIL_ARR_SIZE-1;

// Variables for smart automatic photoperiod functionality
byte light_rampUpBeginHour = 0, light_rampUpBeginMin = 0,
  light_rampDownEndHour = 0, light_rampDownEndMin = 0,
  light_intensity = 0;

// Adds a new value to index 0 of an array of type double, pushing one index forward all previous values
void addToArray( double *Array, double newValue ){
  for( int i = MED_FIL_ARR_SIZE-1; i >= 0; i-- ){
    if( i == 0 ){
      Array[i] = newValue;
    }
    else{
      Array[i] = Array[i - 1];
    }
  }
}

// Copies an entire array of type double and length MED_FIL_ARR_SIZE from src to dest
void copyArray( double* dest, double* src ){
  for( int i = 0; i < MED_FIL_ARR_SIZE; i++ ){
    dest[i] = src[i];
  }
}

// Clears an entire array of length len and type double
void clearArray( char* arr, int len ){
  for( int i = 0; i < len; i++ ){
    arr[i] = '\0';
  }
}

// Returns median-filtered value from Array
double medianFilter( double *Array ){
  double aux;

  copyArray( temp_sortArr, Array );
  
  for( int i = 1; i < MED_FIL_ARR_SIZE; i++ ){
    for( int j = 0; j < MED_FIL_ARR_SIZE - i; j++ ){
      if( temp_sortArr[j] > temp_sortArr[j+1] ){
        aux = temp_sortArr[j];
        temp_sortArr[j] = temp_sortArr[j+1];
        temp_sortArr[j+1] = aux;
      }
    }
  }
  return temp_sortArr[MED_FIL_ARR_SIZE/2];
}

// Sends all 5 readings from sensors in formatted words of width determined by 'width' local variable and terminated by newline char
// Example: 11.11 22.22 33.33 44.44 55.55\n
void sendData(){
  byte i, pad, width = 5;

  pad = width - Serial.print( temp_grow );
  for( i = 0; i < pad; i++ ){
    Serial.print('0');
  }
  Serial.print(' ');

  pad = width - Serial.print( humid_grow );
  for( i = 0; i < pad; i++ ){
    Serial.print('0');
  }
  Serial.print(' ');

  pad = width - Serial.print( temp_room );
  for( i = 0; i < pad; i++ ){
    Serial.print('0');
  }
  Serial.print(' ');

  pad = width - Serial.print( temp_output );
  for( i = 0; i < pad; i++ ){
    Serial.print('0');
  }
  Serial.print(' ');

  pad = width - Serial.print( temp_controlSig ) - Serial.print( '.' );
  for( i = 0; i < pad; i++ ){
    Serial.print('0');
  }

  Serial.print('\n');
}

// Reads values from LM35 temperature sensor, filters them using median filter
// Reads values from DS18B20 temperature sensor
// Reads values from DHT22 relative air humidity and temperature sensor
// Formats and stores everything in variable data_buff
void readSensors(){
  
  // Analog V to Digital Temperature unamplified conversion factor: 5V/1023/10mV°C-¹ = 0.48876
  // Amplifier resistor nominal values: R1 = 2.2k; R2 = 8.2k
  // Thus, theoretic gain: 1+R2/R1 = 4.7273
  // Final conversion factor: 0.1034

  ds18b20.requestTemperatures();

  addToArray( temp_outputArr, analogRead(temp_outputPin)*0.1027 );  // Measured amplifier gain: 4.76
  addToArray( temp_roomArr, ds18b20.getTempC() );

  // Not printing the initial readings is needed for filling up median filter's window
  if( temp_initReads ){
    temp_initReads--;
  }
  else {
    if( temp_initFinished == false ){
      temp_initFinished = true;
      Serial.print( "begin" );
    }

    temp_output = medianFilter( temp_outputArr );
    temp_room = medianFilter( temp_roomArr );

    temp_grow = dht.readTemperature();
    humid_grow = dht.readHumidity();
  }
}

// Interprets and carries out the command stored in cmd_buff string
void cmdRecv(){
  char num[4] = "\0\0\0";
  int addr = 0; // For writing to the EEPROM
  
  // EEPROM important stuff
  //  addr  : content
  //  0 : light_rampUpBeginHour
  //  1 : light_rampUpBeginMin
  //  2 : light_rampDownEndHour
  //  3 : light_rampDownEndMin
  //  4 : temp_ref
  //  5 : rtemp_offset
  
  switch( cmd_buff[0] ){

    // Set the PWM value
    case 't':
      num[0] = cmd_buff[1];
      num[1] = cmd_buff[2];
      num[2] = cmd_buff[3];
      analogWrite( heater_pin, atoi( num ) );
      break;

    // Set the photoperiod
    case 'f':
      byte light_rampUpBeginHour_new, light_rampUpBeginMin_new,
            light_rampDownEndHour_new, light_rampDownEndMin_new;
      
      num[0] = cmd_buff[1];
      num[1] = cmd_buff[2];
      light_rampUpBeginHour_new = atoi( num );
      clearArray( num, 3 );
      
      num[0] = cmd_buff[3];
      num[1] = cmd_buff[4];
      light_rampUpBeginMin_new = atoi( num );
      clearArray( num, 3 );

      num[0] = cmd_buff[5];
      num[1] = cmd_buff[6];
      light_rampDownEndHour_new = atoi( num );
      clearArray( num, 3 );

      num[0] = cmd_buff[7];
      num[1] = cmd_buff[8];
      light_rampDownEndMin_new = atoi( num );

      if( light_rampUpBeginMin_new % 5 != 0 ||
          light_rampDownEndMin_new % 5 != 0 ){
        Serial.print("rThe times for light switching must be multiples of 5 minutes!\n");
        break;
      }else if( light_rampDownEndHour_new <= light_rampUpBeginHour_new ){
        Serial.print("rThe on-cycle of lights must not cross midnight!\n");
        break;
      }else if( light_rampDownEndHour_new - light_rampUpBeginHour_new > 18 ){
        Serial.print("rThe maximum recommended on-cycle of lights is 18 hours.\n");
      }

      light_rampUpBeginHour = light_rampUpBeginHour_new;
      light_rampUpBeginMin = light_rampUpBeginMin_new;
      light_rampDownEndHour = light_rampDownEndHour_new;
      light_rampDownEndMin = light_rampDownEndMin_new;
      
      EEPROM.write( addr, light_rampUpBeginHour );
      addr++;
      EEPROM.write( addr, light_rampUpBeginMin );
      addr++;
      EEPROM.write( addr, light_rampDownEndHour );
      addr++;
      EEPROM.write( addr, light_rampDownEndMin );
      addr++;
      
      break;

    // Set current time
    case 'h':
      num[0] = cmd_buff[1];
      num[1] = cmd_buff[2];
      Clock.setHour( atoi( num ) );
      clearArray( num, 3 );

      num[0] = cmd_buff[3];
      num[1] = cmd_buff[4];
      Clock.setMinute( atoi( num ) );

      Clock.setSecond( 0 );
      break;

    // Set the reference temperature
    case 'r':
      addr = 4;
      num[0] = cmd_buff[1];
      num[1] = cmd_buff[2];
      temp_ref = atoi( num );
      EEPROM.write( addr, temp_ref );
      break;

    // Set light intensity
    case 'l':
      num[0] = cmd_buff[1];
      num[1] = cmd_buff[2];
      num[2] = cmd_buff[3];
      light_intensity = atoi( num );
      break;

    // Checks for the variables in the microcontroller
    case 'c':
      Serial.print( 'r' );
      switch( cmd_buff[1] ){
        // Check the current time
        case 't':
          if( Clock.getHour() < 10 ){
            Serial.print( '0' );
          }
          Serial.print( Clock.getHour() );
          Serial.print( ':' );
          if( Clock.getMinute() < 10 ){
            Serial.print( '0' );
          }
          Serial.print( Clock.getMinute() );
          Serial.print('\n');
          break;

        // Check the photoperiod
        case 'f':
          if( EEPROM.read( 0 ) < 10 )
            Serial.print( '0' );
          Serial.print( EEPROM.read( 0 ) );
          Serial.print( ':' );
          if( EEPROM.read( 1 ) < 10 )
            Serial.print( '0' );
          Serial.print( EEPROM.read( 1 ) );
          Serial.print( " - " );
          if( EEPROM.read( 2 ) < 10 )
            Serial.print( '0' );
          Serial.print( EEPROM.read( 2 ) );
          Serial.print( ':' );
          if( EEPROM.read( 3 ) < 10 )
            Serial.print( '0' );
          Serial.print( EEPROM.read( 3 ) );
          Serial.print( '\n' );
          break;

        // Check light intensity
        case 'l':
          Serial.print( light_intensity );
          Serial.print( '\n' );
          break;

        // Check reference temperature
        case 'r':
          Serial.print( temp_ref );
          Serial.print( '\n' );
          break;

        // Check relative temperature offset
        case 'o':
          Serial.print( rtemp_offset );
          Serial.print( '\n' );
          break;

        default:
          Serial.print( "Command unrecognized. No action invoked.\n" );
          break;
      }
      break;

    default:
      Serial.print( "rCommand unrecognized. No action invoked.\n" );
      break;
  }
}

// Makes lights switch on and off smoothly, slowly increasing (or decreasing) intensity over a 64 minute span
void lightControl(){
  
  byte light_rampDownBeginMin = light_rampDownEndMin == 0 ? 56 : light_rampDownEndMin - 4;
  byte light_rampDownBeginHour = light_rampDownEndMin == 0 ? light_rampDownEndHour - 2 : light_rampDownEndHour - 1;
  
  if( Clock.getHour() == light_rampUpBeginHour && Clock.getMinute() == light_rampUpBeginMin ){
    light_rampingUp = true;
  }
  else if( Clock.getHour() == light_rampUpBeginHour + 1 && Clock.getMinute() == light_rampUpBeginMin + 4 ){
    light_rampingUp = false;
    light_intensity = 255;
  }
  else if( Clock.getHour() == light_rampDownBeginHour && Clock.getMinute() == light_rampDownBeginMin ){
    light_rampingDown = true;
  }
  else if( Clock.getHour() == light_rampDownEndHour && Clock.getMinute() == light_rampDownEndMin ){
    light_rampingDown = false;
    light_intensity = 0;
  }

  analogWrite( light_pin, light_intensity );
  
  if( light_rampingUp ){
    light_intensity += LIGHT_INTENS_INCR;
  }else if( light_rampingDown ){
    //light_intensity = 0;  
  }
}

// Works out controller logic for determining correct PWM value per cycle and applies it to the heater
void tempController(){
  if( temp_grow < temp_ref - 0.5 && temp_controlSig == 0 ){
    rtemp_offset = temp_grow - temp_room;
    EEPROM.write( 5, (byte) rtemp_offset*60 );
  }

  double rtemp_err = temp_ref - temp_grow;
  
  rtemp_target = temp_ref - temp_room - rtemp_offset;
  
  if( rtemp_target > 0 ){
    temp_controlSig = 11.236 * rtemp_target;  // Kp = 1/0.089 experimentally calculated
    temp_controlSig -= light_intensity/12;
    rtemp_intg += rtemp_err;
    temp_controlSig += rtemp_intg * 0.3;
  }
  else{
    temp_controlSig = 0;
    rtemp_intg = 0;
  }

  if( temp_controlSig > 127 ){temp_controlSig = 127;} // Limits control signal to 50% PWM duty cycle, 200W
  if( temp_controlSig < 0 )  {temp_controlSig = 0;}   // No negative values

  analogWrite( heater_pin, temp_controlSig );
}

void setup() {
  // Configure serial communication
  Serial.begin( BAUDRATE );

  // Set specific pin modes
  pinMode( heater_pin, OUTPUT );
  pinMode( temp_outputPin, INPUT );
  pinMode( light_pin, OUTPUT );

  // Load the photoperiod light-switch times
  light_rampUpBeginHour = EEPROM.read( 0 );
  light_rampUpBeginMin = EEPROM.read( 1 );
  light_rampDownEndHour = EEPROM.read( 2 );
  light_rampDownEndMin = EEPROM.read( 3 );

  // Load the reference and offset temperatures for controller logic
  temp_ref = EEPROM.read( 4 );
  rtemp_offset = EEPROM.read( 5 ) / 60.0;

  // Initialize DS18B20
  ds18b20.begin();
  
  // Initialize the RTC object
  Wire.begin();

  // Set lights on so there's no interference with smooth transitioning at the moment of booting up
  if( Clock.getHour() == light_rampUpBeginHour && Clock.getMinute() >= light_rampUpBeginMin ||
      Clock.getHour() > light_rampUpBeginHour && Clock.getHour() < light_rampDownEndHour ||
      Clock.getHour() == light_rampDownEndHour && Clock.getMinute() <= light_rampDownEndMin ){
    light_intensity = 255;
    analogWrite( light_pin, light_intensity );
  }

  // Initialize the DHT22 digital interface
  dht.begin();
}

void loop() {
  // Check serial port for incoming commands
  while( Serial.available() > 0 ){
    msg_byte = Serial.read();
    if( msg_byte != '\n' ){
      cmd_buff[cmd_i] = msg_byte;
      cmd_i++;
    }
    else{
      cmdRecv();
      clearArray( cmd_buff, 10 );
      cmd_i = 0;
    }
  }

  // Get temperature and relative humidity readings once every TEMP_MEASR_DELAY_S seconds
  if( Clock.getSecond() % TEMP_MEASR_DELAY_S == 0 ){
    if( temp_measrUpdated == false ){
      readSensors();
      temp_measrUpdated = true;
    }
  }
  else
    temp_measrUpdated = false;

  // Apply temperature controller logic once every TEMP_MEASR_DELAY_S seconds
  if( Clock.getSecond() % TEMP_MEASR_DELAY_S == 1 ){
    if( temp_initFinished ){
      if( temp_controlled == false){
        tempController();
        temp_controlled = true;
      }
    }
  }
  else
    temp_controlled = false;
  
  // Turn lights on or off smoothly
  if( Clock.getSecond() % 15 == 2 ){
    if( light_controlled == false ){
      lightControl();
      light_controlled = true;
    }
  }
  else
    light_controlled = false;

  // Send data through serial port every TEMP_MEASR_DELAY_S seconds
  if( Clock.getSecond() % TEMP_MEASR_DELAY_S == 3 ){
    if( temp_initFinished ){
      if( !dataSent ){
        sendData();
        dataSent = true;
      }
    }
  }
  else
    dataSent = false;

  delay(500); 
}
