/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop! 

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution

 Modified by Nico Lingg (2017) for the Bluetooth LE UART Example
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_SHT31.h" //librería para sensor T°/H

#include "BluefruitConfig.h" //librería para módulo BT
#include "Wire.h"

#include <Adafruit_NeoPixel.h> //librería para NeoPixels
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


/*Configuración de LEDs*/

// PIN de salida a NeoPixels
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// Cantidad de NeoPixels que hay
#define NUMPIXELS 1

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
/*------------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/



/*Configuración de sensor y módulo BT*/

/* SHT31 dirección por defecto */
#define SH31_ADDR 0x44


/* Comunicación UART */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN); 



/* Create Adafruit_SHT31 object - !! Adafruit_SHT31 Library required !! */
Adafruit_SHT31 sht31 = Adafruit_SHT31();


/* Globale enum (= states) for state machines */
enum state_LED {LED_OFF, LED_ON_WAIT, LED_ON, LED_OFF_WAIT} s_LED = LED_OFF;
enum state_Sensor {SENSOR_WAIT, SENSOR_READ_DATA, SENSOR_SEND_DATA} s_Sensor = SENSOR_WAIT;

/* Flags for LED 'State Machine' */
int FlagLED=0;

/* Counter for Sensor 'state Machine' */
int Counter=0; // state machine every 10 ms -> 3 secs -> Counter = 300;

/* Global Variables */
float t;
float h;
int temperature;
int humidity;


/* A Small Helper */
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  This function sets up the hardware, the BLE module and 
            the SHT31 (this function is called automatically on startup)
*/
/**************************************************************************/
void setup(void)
{    pixels.begin();
  /* disable global interrupts */ 
    
 //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  /* Initialize serial port for debugging */ 

  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Chequear que esté en CMD y la conexión sea correcta?"));
  }
  Serial.println( F("OK!") );

  /* Initialize timer for interrupt - 100Hz */
  //timer1_init();
  
  /* Initialize digital pin LED_BUILTIN as an output and set it to low */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Test - delete
 pinMode(2, OUTPUT);
 pinMode(3, OUTPUT);
 pinMode(4, OUTPUT);
  
  /* Iniciar sensor*/
  Serial.println("Inicializando SHT31");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("No se encontró el sensor de T/H");
    while (1) delay(1);
  }


  /* Iniciar módulo */
  Serial.print(F("Iniciando el módulo Bluetooth "));
  
  if ( FACTORYRESET_ENABLE )
  {
    /* Reseteo */
    Serial.println(F("Factory reset: "));
    if ( ! ble.factoryReset() ){ 
      delay(200);
      //intentar de nuevo
      if(!ble.factoryReset()){
        error(F("No se pudo realizar el reset"));
      }
    }
  }

  /* Se elimina el comando echo del BT ya que no se necesita */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Conexión del usuario con el módulo */
  Serial.println(F("Por favor, conectarse a la app"));
  Serial.println();

  ble.verbose(false);
  
  /* Espera a que el usuario se conecte. Se indica con LED. */
  while (! ble.isConnected()) {
    
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
  }
  digitalWrite(LED_BUILTIN, LOW);

  /* LED Activity command is only supported from 0.6.6 */
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

}

/**************************************************************************/
/*!
    @brief  Main loop 
*/
/**************************************************************************/
void loop(void)
{
  SM_LED();
  SM_SENSOR();
  delay(250);
}

/**************************************************************************/
/*!
    @brief  'State Machine' for build-in LED - this function turns on and 
             off the built-in LED
*/
/**************************************************************************/
void SM_LED()
{    
   /* Check for incoming characters */
   digitalWrite(2, digitalRead(2) ^ 1); // toggle LED pin
    ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK") == 0){
      // no data
    }
    
    /* Some data was found, its in the buffer */
    Serial.print(F("[Recv] ")); Serial.println(ble.buffer);

    /* Check what was sent and set LED (1==LED_ON, 0==LED_OFF) */
    if(strcmp(ble.buffer, "LED_ON") == 0){
      FlagLED = 1;
    }
    if(strcmp(ble.buffer, "LED_OFF") == 0){
      
      FlagLED = 0;
    }
    /* check response status */
    ble.waitForOK();
    
    switch(s_LED)
    {
      case LED_OFF:
        digitalWrite(LED_BUILTIN, LOW);
        s_LED = LED_ON_WAIT;
        break;

      case LED_ON_WAIT:
        if(FlagLED==1) s_LED=LED_ON;
        break;
             
      case LED_ON:
        digitalWrite(LED_BUILTIN, HIGH);
        s_LED = LED_OFF_WAIT;
        break;

      case LED_OFF_WAIT:
        if(FlagLED==0) s_LED=LED_OFF;
        break; 
    }
  
}

/**************************************************************************/
/*!
    @brief  'State Machine' for the SHT31 temperature and humidity sensor
             This function reads the sensor values and sends it via
             Bluetooth LE to the Central
*/
/**************************************************************************/
void SM_SENSOR()
{

  switch(s_Sensor)
  { /*Después de que el contador llega a 3, el sensor lee los datos. (Este proceso, debido a lo indicado en la librería del SHT31, tarda aprox 3000ms*/
    case SENSOR_WAIT:
      if(Counter>=3) 
      {
        s_Sensor = SENSOR_READ_DATA;
        Counter=0; 
      }
      break;

    case SENSOR_READ_DATA:
        t = sht31.readTemperature();
        h = sht31.readHumidity();
        temperature = (int)(t*100);
        humidity = (int)(h*100);
        s_Sensor = SENSOR_SEND_DATA; 

        /* LEDs indicadores de temperatura*/
        if (t>30)
{
  pixels.clear(); // Se apagan los LEDs

  for(int i=0; i<NUMPIXELS; i++) { 
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));

    pixels.show();   // Actualiza el color
    delay(DELAYVAL); // Pausa
  }
}
else  {  
    pixels.clear(); 
  for(int i=0; i<NUMPIXELS; i++) {.
  pixels.clear(); '
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));

    pixels.show();   

    delay(DELAYVAL); 
  } }
      break;

    case SENSOR_SEND_DATA:
      sensor_send_data();
      s_Sensor = SENSOR_WAIT;
      break;   
  }
  /* Incremento contador */
  Counter++;
}

/**************************************************************************/
/*!
    @brief  This funtion sends the sensor data via Bluetooth to the Qt-app
*/
/**************************************************************************/
void sensor_send_data(){
  
 if (ble.isConnected())
 {
    /* Se envian los datos por puerto serie */
    ble.print("AT+BLEUARTTX=");
    ble.print("Temperatura");
    ble.print(t);
    //Serial.print("°C");
    ble.print("Humedad ");
    ble.print(h);
    //Serial.print("%");
    ble.println(" - ");



    
    /* check response status */
    if (! ble.waitForOK() ) {
        Serial.println(F("Failed to send?"));
      }
 }
 
} 
