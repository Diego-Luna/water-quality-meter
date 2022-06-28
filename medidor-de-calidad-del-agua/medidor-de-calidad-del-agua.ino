//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Modificar
// Modify

String Nombre  = "Diego_Test_1"; // maximo 20 caracteres, sin espacios
String posicion_x = "19.707085";
String posicion_y = "-98.460370";
String Comunidad_o_institucion  = "EL_salto_GDL_MEX"; // maximo 20 caracteres, sin espacios

// El timpo es de 0 - 24
int hora_de_instalacion = 16;
int hora_de_medicion_1 = 12;
int hora_de_medicion_2 = 24;

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Calibracion
#define Offset 0.00            //deviation compensate
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "ArduinoLowPower.h"

// -> Lora
#include <SPI.h>
#include <LoRa.h>

int pit_transistor = 1;

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

byte time_status = 0;

// -> pH
#define SensorPin_ph A0            //pH meter Analog output to Arduino Analog Input 0
//#define Offset 0.00            //deviation compensate
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
#define sensorPin_tds A0

int sensorValue = 0;
float tdsValue = 0;
float Voltage = 0;



// -> Water temperature
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2

float temperature = 25;

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
  LoRa.setSyncWord(0xF3);

  pinMode(pit_transistor, OUTPUT);

  ft_apagar_sensores(true);
  pinMode(LED, OUTPUT);
  //  pinMode(TdsSensorPin, INPUT);

  sensors.begin();
  horaTime = hora_de_instalacion;
}

void loop(void)
{
  //  ft_time();
  ft_apagar_sensores(true);
  ft_get_water_temperature();
  ft_get_turviedad();
  ft_get_ph();
  ft_get_tds();
  ft_send_data();
  ft_time();
  ft_apagar_sensores(false);
  LowPower.sleep(sleep_time);
  //  delay(45000);
}
// --- Time
void ft_time()
{

  myTime = millis();

  unsigned long int days = myTime  / day_ms ;                                //number of days
  unsigned long int hours = (myTime % day_ms) / hour_ms;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  unsigned long int minutes = ((myTime % day_ms) % hour_ms) / minute_ms ;         //and so on...
  unsigned long int seconds = (((myTime % day_ms) % hour_ms) % minute_ms) / second_ms;

  horaTime += hours;

  if (horaTime >= 24) horaTime = 0;


  //  sleep_time = ft_get_sleep_time()  - (2 * second_ms);
  sleep_time = ft_get_sleep_time();

  Serial.println();
  Serial.print("----> Timepo : ");
  Serial.print(sleep_time);
  Serial.println();
}

int ft_get_sleep_time()
{
  if (time_status == 0)
  {
    if (hora_de_instalacion <= hora_de_medicion_1)
    {
      time_status = 1;
      return (ft_hour_in_ms(hora_de_medicion_1 - hora_de_instalacion));
    }
    if (hora_de_instalacion <= hora_de_medicion_2)
    {
      time_status = 2;
      return (ft_hour_in_ms(hora_de_medicion_2 - hora_de_instalacion));
    }
    if (hora_de_instalacion > hora_de_medicion_2)
    {
      time_status = 1;
      return (ft_hour_in_ms((24 - hora_de_instalacion) + hora_de_medicion_1));
    }
  }
  else if (time_status == 1) //de 1 a 2
  {
    time_status = 2;
    return (ft_hour_in_ms(hora_de_medicion_2 - hora_de_medicion_1));
  }
  else if (time_status == 2) //de 2 a 1
  {
    time_status = 1;
    return (ft_hour_in_ms((24 - hora_de_medicion_2) + hora_de_medicion_1));
  }
}

int ft_hour_in_ms(int myHour) {
  return (myHour * hour_ms);
}

// --- Lora

void ft_apagar_sensores(bool estado)
{
  if (estado == true) {
    digitalWrite(pit_transistor, HIGH);
  }
  else
  {
    digitalWrite(pit_transistor, LOW);
  }
}

void  ft_send_data()
{
  //  char *space = ",";
  String space = ",";
  String clean = "000";
  String send_values;

  pHValue = 7.77;
  tdsValue = 200;
  sensorValue_turviedad = 300;
  temperature = 25.20;

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

  send_values = Nombre + space + Comunidad_o_institucion + space + posicion_x + space + posicion_y + space;
  send_values = send_values + value_pH + space;
  send_values = send_values + value_TDS + space;
  send_values = send_values + value_Turviedad + space;
  send_values = send_values + value_Water_Tem + space + clean;

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
  sensorValue = analogRead(sensorPin_tds);
  Voltage = sensorValue * 5 / 1024.0; //Convert analog reading to Voltage
  tdsValue = (133.42 / Voltage * Voltage * Voltage - 255.86 * Voltage * Voltage + 857.39 * Voltage) * 0.5; //Convert voltage value to TDS value
  //  SERIAL.print("TDS Value = ");
  //  SERIAL.print(tdsValue);
  //  SERIAL.println(" ppm");
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
  sensorValue_turviedad = analogRead(A2);
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
    pHArray[pHArrayIndex++] = analogRead(SensorPin_ph);
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
