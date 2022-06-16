//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Modificar
// Modify

String Name  = "Diego_Test_1"; // maximo 20 caracteres, sin espacios
String position_x = "19.707085";
String position_y = "-98.460370";
String Comunity_or_institution  = "EL_salto_GDL_MEX"; // maximo 20 caracteres, sin espacios

// El timpo es de 0 - 24
int time_start = 14;
int time_send_1 = 3;
int time_send_2 = 8;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "ArduinoLowPower.h"

// -> Lora
#include <SPI.h>
#include <LoRa.h>

// -> Hora

unsigned long myTime;
unsigned long oldTime;
unsigned long sleep_time;
int horaTime;
int hora;
int minutos;
int segundos;

long day_ms = 86400000; // 86400000 milliseconds in a day
long hour_ms = 3600000; // 3600000 milliseconds in an hour
long minute_ms = 60000; // 60000 milliseconds in a minute
long second_ms =  1000;

bool on_start = false;

// -> pH
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
float pHValue, voltage;

// -> Turviedad
int sensorValue_turviedad = 0;
float voltage_turviedad = 0.0;

// -> TDS
#define TdsSensorPin A2
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

// -> Water temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void setup(void)
{

  Serial.begin(9600);
  Serial.println("LoRa Sender - water quality meter");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  pinMode(LED, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  sensors.begin();
  horaTime = time_start;
}

void loop(void)
{
  ft_time();
  ft_get_water_temperature();
  ft_get_turviedad();
  ft_get_ph();
  ft_get_tds();
  ft_send_data();
  ft_time();
  if (on_start == false)
  {
    on_start = true;
    LowPower.sleep(10000);
  } else {
    delay(10000);
  }
  //  LowPower.sleep(10000);
  //  delay(10000);
}
// --- Time
void ft_time()
{
  //  unsigned long myTime;
  //  unsigned long oldTime;
  //  unsigned long sleep_time;
  //  int horaTime;
  //  int hora;
  //  int minutos;
  //  int segundos;
  //
  //  long day_ms = 86400000; // 86400000 milliseconds in a day
  //  long hour_ms = 3600000; // 3600000 milliseconds in an hour
  //  long minute_ms = 60000; // 60000 milliseconds in a minute
  //  long second_ms =  1000;

  //  int time_start = 14;
  //  int time_send_1 = 12;
  //  int time_send_2 = 24;

  myTime = millis();

  unsigned long int days = myTime  / day_ms ;                                //number of days
  unsigned long int hours = (myTime % day_ms) / hour_ms;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  unsigned long int minutes = ((myTime % day_ms) % hour_ms) / minute_ms ;         //and so on...
  unsigned long int seconds = (((myTime % day_ms) % hour_ms) % minute_ms) / second_ms;

  horaTime += hours;

  if (horaTime >= 24) horaTime = 0;


  sleep_time = (time_send_2 - time_send_1) * hour_ms  - (2 * second_ms);

  Serial.println();
  Serial.print("--> Timepo : ");
  Serial.print(time_send_2 - time_send_1);
  Serial.print(" H, ");
  Serial.print(sleep_time);
  Serial.print(" ms, ");
  Serial.print(seconds);
  Serial.print(" s, ");
  Serial.print(minutes);
  Serial.print(" minutes.");
  Serial.println();
}

// --- Lora

void  ft_send_data()
{
  //  char *space = ",";
  String space = ",";
  String send_values;

  pHValue = 7.77;
  tdsValue = 200;
  sensorValue_turviedad = 300;
  temperature = 25.0;

  String value_pH = String(pHValue, 2);
  String value_TDS = String(tdsValue, 0);
  String value_Turviedad = String(sensorValue_turviedad);
  String value_Water_Tem = String(temperature, 2);

  Serial.println();
  Serial.print("->       data: ");
  Serial.print(pHValue);
  Serial.print(",");
  Serial.print(tdsValue);
  Serial.print(",");
  Serial.print(sensorValue_turviedad);
  Serial.print(",");
  Serial.println(temperature);

  Serial.print("-> sting data: ");
  Serial.print(value_pH);
  Serial.print(",");
  Serial.print(value_TDS);
  Serial.print(",");
  Serial.print(value_Turviedad);
  Serial.print(",");
  Serial.println(value_Water_Tem);

  send_values = Name + space + position_x + space + position_y + space;
  send_values = send_values + value_pH + space;
  send_values = send_values + value_TDS + space;
  send_values = send_values + value_Turviedad + space;
  send_values = send_values + value_Water_Tem + space;

  Serial.print("--> send data: ");
  Serial.println(send_values);

  //  -- Send lora
  LoRa.beginPacket();
  LoRa.print(send_values);
  LoRa.endPacket();

}

// --- Water temperature - DS18B20
void ft_get_water_temperature()
{
  Serial.print(" Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperature readings
  Serial.println("DONE");
  /********************************************************************/
  Serial.print("Temperature is: ");
  temperature = sensors.getTempCByIndex(0);
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
  // You can have more than one DS18B20 on the same bus.
  // 0 refers to the first IC on the wire
}

// --- TDS
void ft_get_tds()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = ft_getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}

int ft_getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

// --- Turviedad
void  ft_get_turviedad()
{
  sensorValue_turviedad = analogRead(A1);
  voltage_turviedad = sensorValue_turviedad * (5.0 / 1024.0);
}

// --- PH - SEN0161 - analog pH meter

void  ft_get_ph()
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  //  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = ft_avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    Serial.print("Voltage:");
    Serial.print(voltage, 2);
    Serial.print("    pH value: ");
    Serial.println(pHValue, 2);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    printTime = millis();
  }
}

double ft_avergearray(int* arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}
