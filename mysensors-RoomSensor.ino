#include <MySensor.h>
#include "arduino.h"
#include <DHT.h>
#include "SparkFunBME280.h"

//configuration
//#define HAS_NTC
#define HAS_LDR
//#define HAS_BME280
#define HAS_DHT

//BME280 related:
BME280 bmeSensor;


// DHT Sensor related:
#define HUMIDITY_SENSOR_DIGITAL_PIN 3
DHT dht;
boolean metric = true; 

// Sensor ID's
#define DHT_Temp_ID 1
#define DHT_Humidity_ID 2
#define NTC_Temp_ID 3        // depending on ntc
#define LDR_Light_ID 3       // or ldr
#define BME_Temp_ID 4
#define BME_Humidity_ID 5
#define BME_Pressure_ID 6


// mysensors related
MySensor gw;
//MyMessage msg(Moisture_ID,V_LEVEL);
MyMessage DHTTempMsg(DHT_Temp_ID,V_TEMP);
MyMessage DHTHumMsg(DHT_Humidity_ID,V_HUM);
#ifdef HAS_NTC
MyMessage NTCTempMsg(NTC_Temp_ID,V_TEMP);
#endif
#ifdef HAS_LDR
MyMessage LDRLightMsg(LDR_Light_ID,V_LIGHT_LEVEL);
#endif
#ifdef HAS_BME280
MyMessage BMETempMsg(BME_Temp_ID,V_TEMP);
MyMessage BMEHumMsg(BME_Humidity_ID,V_HUM);
MyMessage BMEBaroMsg(BME_Pressure_ID,V_PRESSURE);
#endif
// local defs
unsigned long timediff(unsigned long t1, unsigned long t2);
#ifdef HAS_NTC
int NTC_ADC2Temperature(unsigned int adc_value);
#endif
int getBatteryLevel(void);


extern volatile unsigned long timer0_millis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial)
  {
   ; // wait for serial port to connect. Needed for native USB
  }
  Serial.print("Serial init\r\n"); 

  //pinMode(A3,INPUT);
  //while(1)
  //{
  //  getBatteryLevel();
  //  delay(200);
  //}

  // mysensors init:
  gw.begin();
  Serial.print("GW Begin\r\n");
  gw.sendSketchInfo("Room Sensor Node", "1.0"); 
  Serial.print("Sketch Info\r\n");
  
  //register the sensors to the gateway
  gw.present(DHT_Temp_ID , S_TEMP);
  gw.present(DHT_Humidity_ID , S_HUM);
  #ifdef HAS_NTC
  gw.present(NTC_Temp_ID , S_TEMP);
  #endif
  #ifdef HAS_LDR
  gw.present(LDR_Light_ID , S_LIGHT_LEVEL);
  #endif
  #ifdef HAS_BME280
  gw.present(BME_Temp_ID , S_TEMP);
  gw.present(BME_Humidity_ID , S_HUM);
  gw.present(BME_Pressure_ID , S_BARO);
  #endif
  
  // ------ DHT11 init -----
  //Pin 4 is the power source for the DHT
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  //
  //gw.sleep(2*1000); // 2 sec
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  dht.getTemperature();
  delay(2000);
  Serial.print("DHT init\r\n");

  // ---- NTC / LDR Init ----
  pinMode(A1,INPUT);
  pinMode(A2,OUTPUT);
  digitalWrite(A2,LOW);
  Serial.print("NTC/LDR init\r\n");

  #ifdef HAS_BME280
  // ------ BME280 Init ------
  bmeSensor.settings.commInterface = I2C_MODE;
  bmeSensor.settings.I2CAddress = 0x77;
  bmeSensor.settings.runMode = 3; //Normal mode
  bmeSensor.settings.tStandby = 0;
  bmeSensor.settings.filter = 0;
  bmeSensor.settings.tempOverSample = 1;
  bmeSensor.settings.pressOverSample = 1;
  bmeSensor.settings.humidOverSample = 1;
  Serial.println(bmeSensor.begin(), HEX);
  Serial.print("Displaying ID, reset and ctrl regs\n");
  Serial.print("ID(0xD0): 0x");
  Serial.println(bmeSensor.readRegister(BME280_CHIP_ID_REG), HEX);
  Serial.print("Reset register(0xE0): 0x");
  Serial.println(bmeSensor.readRegister(BME280_RST_REG), HEX);
  Serial.print("ctrl_meas(0xF4): 0x");
  Serial.println(bmeSensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  Serial.print("ctrl_hum(0xF2): 0x");
  Serial.println(bmeSensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

  Serial.print("\n\n");
  
  Serial.print("BME280 init\r\n"); 
  #endif
  
  //  ---- Battery Input ----
  pinMode(A3,INPUT);
  
}


void loop() {
  
  static uint8_t DoSleep = 1;
  
  // static unsigned long TempTime = 0;
  // if (timediff(TempTime, millis()) > 4000 )
  {
    
      digitalWrite(A2, HIGH);
     // TempTime = millis();

    #ifdef HAS_NTC
      Serial.println("ntc:");
      int ntc = analogRead(A1);
      int temp = NTC_ADC2Temperature(ntc);
      Serial.println(ntc);
      Serial.println(temp);
      //send the measured value
      gw.send(NTCTempMsg.set(((float)temp)/10,1));
    #endif
    #ifdef HAS_LDR
      Serial.println("ldr:");
      int ldr = analogRead(A1);
      Serial.println(ldr);
      //send the measured value
      gw.send(LDRLightMsg.set(ldr, 1) );
    #endif
      
      digitalWrite(A2, LOW);
      Serial.println();
      gw.sendBatteryLevel(getBatteryLevel());
      Serial.println();
  }
  // dht test:
  //static unsigned long DHTTime = 0;
  //if(timediff(millis(), DHTTime) > 4000) // every ~ 2sec
  {
    //DHTTime = millis();
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
    }else{
      if (!metric) {
        temperature = dht.toFahrenheit(temperature);
      }
      gw.send(DHTTempMsg.set(temperature, 1));
      Serial.println();
      Serial.print("DHT Temp: ");
      Serial.println(temperature); 
    }
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
        Serial.println("Failed reading humidity from DHT");
    }
    else{
      gw.send(DHTHumMsg.set(humidity, 1));
      Serial.println();
      Serial.print("DHT Hum: ");
      Serial.println(humidity);
    }
  }

  #ifdef HAS_BME280
  //static unsigned long BMETime = 0;
  //if(timediff(millis(), BMETime) > 4000) // every ~ 4sec
  {
    //BMETime=millis();
  
    float temp = bmeSensor.readTempC(); 
    gw.send(BMETempMsg.set(temp, 1));
    Serial.println();
    Serial.print("Temperature: ");
    Serial.print(temp, 2);
    Serial.println(" degrees C");
   
    Serial.print("Temperature: ");
    Serial.print(bmeSensor.readTempF(), 2);
    Serial.println(" degrees F"); 
      
    float pressure = bmeSensor.readFloatPressure();
    gw.send(BMEBaroMsg.set(pressure, 1));
    Serial.println();
    Serial.print("Pressure: ");
    Serial.print(pressure, 2);
    Serial.println(" Pa");
      
    Serial.print("Altitude: ");
    Serial.print(bmeSensor.readFloatAltitudeMeters(), 2);
    Serial.println("m");
  
    Serial.print("Altitude: ");
    Serial.print(bmeSensor.readFloatAltitudeFeet(), 2);
    Serial.println("ft"); 
  
    float humidity = bmeSensor.readFloatHumidity();
    gw.send(BMEHumMsg.set(humidity, 1));
    Serial.println();
    Serial.print("%RH: ");
    Serial.print(humidity, 2);
    Serial.println(" %");
    
    Serial.println();
  }
  #endif

  #define SLEEP_TIME (2UL*60UL*1000UL)    // 2min * 60sec * 1000ms
  //#define SLEEP_TIME (2UL*1000UL)    // 2sec * 1000ms
  
  //go into power down mode: 
  gw.sleep(SLEEP_TIME);  

  // adjust the value for millis():
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += SLEEP_TIME;  
  SREG = oldSREG;
}

unsigned long timediff(unsigned long t1, unsigned long t2)
{
    signed long d = (signed long)t1 - (signed long)t2;
    if(d < 0) d = -d;
    return (unsigned long) d;
}


#ifdef HAS_NTC
//http://www.sebulli.com/ntc/index.php?lang=de&points=64&unit=0.1&resolution=10+Bit&circuit=pullup&resistor=10000&r25=10000&beta=3950&tmin=-30&tmax=50
//https://www.amazon.de/SODIAL-Stueck-Temperaturmessung-Thermistoren-MF52-103/dp/B00HUHC4UU/ref=sr_1_3?ie=UTF8&qid=1465509339&sr=8-3&keywords=ntc

/**
* Die NTC Tabelle, bestehend aus 65 Temperaturstützpunkten.
* Einheit:0.1 °C
*
*/
int NTC_table[65] = {
  1921, 1607, 1293, 1127, 1016, 933, 866, 811, 
  763, 722, 685, 652, 621, 593, 567, 543, 520, 
  498, 477, 458, 439, 421, 403, 386, 370, 354, 
  338, 323, 308, 293, 278, 264, 250, 236, 222, 
  208, 194, 181, 167, 153, 139, 125, 111, 97, 
  83, 68, 53, 37, 22, 5, -11, -29, -47, -66, 
  -87, -108, -132, -157, -186, -218, -256, 
  -302, -364, -460, -556
};
 

/**
* \brief    Konvertiert das ADC Ergebnis in einen Temperaturwert.
*
*           Mit p1 und p2 wird der Stützpunkt direkt vor und nach dem
*           ADC Wert ermittelt. Zwischen beiden Stützpunkten wird linear
*           interpoliert. Der Code ist sehr klein und schnell.
*           Es wird lediglich eine Ganzzahl-Multiplikation verwendet.
*           Die Division kann vom Compiler durch eine Schiebeoperation.
*           ersetzt werden.
*
*           Im Temperaturbereich von -30°C bis 50°C beträgt der Fehler
*           durch die Verwendung einer Tabelle 0.144°C
*
* \param    adc_value  Das gewandelte ADC Ergebnis
* \return              Die Temperatur in 0.1 °C
*
*/
int NTC_ADC2Temperature(unsigned int adc_value){
 
  int p1,p2;
  /* Stützpunkt vor und nach dem ADC Wert ermitteln. */
  p1 = NTC_table[ (adc_value >> 4)  ];
  p2 = NTC_table[ (adc_value >> 4)+1];
 
  /* Zwischen beiden Punkten linear interpolieren. */
  return p1 - ( (p1-p2) * (adc_value & 0x000F) ) / 16;
};

#endif

// ------------- BATTERY --------------

#define BAT_AVG_NUM 8
#define BAT100  1500
#define BAT0    800

int ReadBandgap()
{
    uint8_t low, high;
    uint8_t reference = DEFAULT;
    uint8_t pin = 14;
    ADMUX = (reference << 6) | (pin);

    // settle
    delayMicroseconds(200);    
    // start the conversion
    //sbi(ADCSRA, ADSC);
    ADCSRA |= 1 << ADSC;
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));

    low  = ADCL;
    high = ADCH;

    return (high << 8) | low;
}

int getBatteryLevel()
{
  // Read the adc 
  uint32_t bandgap = 341; // ReadBandgap(); // 1.1 Voltage value
  
  //for(int i = 0; i < 2; i++)
  //{

    bandgap = ReadBandgap();
    Serial.print("Bg adc: ");
    Serial.println(bandgap);
  //}  
  
  //Serial.print("Bg adc: ");
  //Serial.println(bandgap);

  uint32_t ArefVoltage = (1100UL * 1024UL) / (bandgap);
  Serial.print("Aref: ");
  Serial.println(ArefVoltage);
  
  uint32_t adc = 0; 
  
  for(int i = 0; i < BAT_AVG_NUM; i++){
    uint32_t temp = analogRead(A3);
    //Serial.print("temp: ");
    //Serial.println(temp);
    adc += temp;
  }
  Serial.print("Bat adc: ");
  Serial.println(adc);

  // Calculate the voltage in mV
  // Reference is 3.3V 
  //uint32_t Voltage = (330 * adc) / (1024*BAT_AVG_NUM);
  // 1.1V bandgap reference calibration
  uint32_t Voltage = (1100UL * adc) / (bandgap * BAT_AVG_NUM);
  Serial.print("Voltage [mV]: ");
  Serial.println(Voltage);

  // Clamp at the Borders
  if(Voltage < BAT0) Voltage = BAT0;
  if(Voltage > BAT100) Voltage = BAT100;
  
  // calculate level
  // let 1.5V be 100%
  // let 0.8V be 0%
  uint32_t level = (100*(Voltage-BAT0))/(BAT100-BAT0);
  Serial.print("BAT LEVEL [%]: ");
  Serial.println(level);

  return (int)level;
}


