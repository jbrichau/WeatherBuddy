#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h>
#include "Adafruit_MPL3115A2.h"
#include <PietteTech_DHT.h>

#define DHTTYPE  DHT22
#define DHTPIN   D4
#define WDIR     A0
#define RAIN     D2
#define WSPEED   D3

//Global Variablesd
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond;  //The millis counter to see when a second rolls by
byte seconds;     //When it hits 60, increase the current minute
byte seconds_2m;  //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes;     //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120];          //120 bytes to keep track of 2 minute average
int winddiravg[120];           //120 ints to keep track of 2 minute average
double windgust_10m[10];        //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60];   //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0;             // [0-360 instantaneous wind direction]
double windspeedkmh = 0;     // [kmh instantaneous wind speed]
double windgustkmh = 0;      // [kmh current wind gust, using software specific time period]
int windgustdir = 0;         // [0-360 using software specific time period]
double windspdkmh_avg2m = 0; // [kmh 2 minute average wind speed kmh]
int winddir_avg2m = 0;       // [0-360 2 minute average wind direction]
double windgustkmh_10m = 0;  // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0;     // [0-360 past 10 minutes wind gust direction]
double rainmm = 0;           // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
long lastWindCheck = 0;
volatile double dailyrainmm = 0; // [rain inches so far today in local time]

double humidity = 0;
double tempc = 0;
double pascals = 0;
double baroTemp = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
PietteTech_DHT DHT(DHTPIN, DHTTYPE);

SYSTEM_THREAD(ENABLED);
ApplicationWatchdog wd(60000, System.reset);

//---------------------------------------------------------------
void setup()
{
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP);   // input from wind meters rain gauge sensor

  while(! baro.begin()) {
    Serial.println("Could not start sensor");
    delay(1000);
  }

  DHT.begin();

  attachInterrupt(RAIN, rainIRQ, FALLING);
  attachInterrupt(WSPEED, wspeedIRQ, FALLING);

  Particle.variable("winddir", winddir);
  Particle.variable("winddir2mavg", winddir_avg2m);
  Particle.variable("windspeed", windspeedkmh);
  Particle.variable("windgust", windgustkmh);
  Particle.variable("windspeed2mavg", windspdkmh_avg2m);
  Particle.variable("rainmm", rainmm);
  Particle.variable("temp", tempc);
  Particle.variable("humidity", humidity);
  Particle.variable("barotemp", baroTemp);
  Particle.variable("pressure", pascals);

  Particle.function("reset", resetme);

  seconds = 0;
  for (int i = 0; i < 10; i++)
  {
    windgust_10m[i] = 0;
    windgustdirection_10m[i] = 0;
  }
  lastSecond = millis();
  Time.zone(2);
}

String weatherDataJson() {
  DynamicJsonDocument doc(1024);
  char output[1024];

  doc["winddir"] = winddir;
  doc["windkmh"] = windspeedkmh;
  doc["windgustkmh"] = windgustkmh;
  doc["windgustdir"] = windgustdir;
  doc["winddir2mavg"] = winddir_avg2m;
  doc["windkmh2mavg"] = windspdkmh_avg2m;
  doc["windgustkmh10m"] = windgustkmh_10m;
  doc["windgustdir10m"] = windgustdir_10m;
  doc["60mrainmm"] = rainmm;
  doc["dailyrainmm"] = dailyrainmm;
  doc["tempc"] = tempc;
  doc["barotempc"] = baroTemp;
  doc["humidity"] = humidity;
  doc["pressure"] = pascals /100;
  
  serializeJson(doc,output);
  return output;
}
//---------------------------------------------------------------
void loop()
{
  //Keep track of which minute it is
  if (millis() - lastSecond >= 1000)
  {
    lastSecond += 1000;
    //Take a speed and direction reading every second for 2 minute average
    if (++seconds_2m > 119)
      seconds_2m = 0;

    if (++seconds > 59)
    {
      seconds = 0;

      if (++minutes > 59)
        minutes = 0;
      if (++minutes_10m > 9)
        minutes_10m = 0;

      rainHour[minutes] = 0;         //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
    }

    //Get readings from all sensors
    getWeather();

    if (seconds == 0)
      Particle.publish("weatherdata", weatherDataJson(), PRIVATE);

    // Reset windgust and dailyrain at midnight
    if (Time.hour() == 0 && Time.minute() == 0) {
      windgustkmh = 0;
      dailyrainmm = 0;
    } 
  }
}


//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  //Wind Vains may vary in the values they return. To get exact wind direction,
  //it is recomended that you AnalogRead the Wind Vain to make sure the values
  //your wind vain output fall within the values listed below.
  if (adc > 2270 && adc < 2290)
    return (0); //North
  if (adc > 3220 && adc < 3299)
    return (1); //NE
  if (adc > 3890 && adc < 3999)
    return (2); //East
  if (adc > 3780 && adc < 3850)
    return (3); //SE

  if (adc > 3570 && adc < 3650)
    return (4); //South
  if (adc > 2790 && adc < 2850)
    return (5); //SW
  if (adc > 1580 && adc < 1610)
    return (6); //West
  if (adc > 1930 && adc < 1950)
    return (7); //NW

  return (-1); // error, disconnected?
}

//Returns the instantaneous wind speed
double get_wind_speed()
{
  double deltaTime = (millis() - lastWindCheck) / 1000.0; 
  double windSpeed = (double)windClicks / deltaTime;

  windClicks = 0;
  lastWindCheck = millis();

  windSpeed *= 2.40; // see datasheet

  return (windSpeed);
}

void getWeather()
{
  int result = DHT.acquireAndWait(2000);
  switch (result) {
    case DHTLIB_OK:
      humidity = DHT.getHumidity();
      tempc = DHT.getCelsius();
      // dewpoint = DHT.getDewPointSlow();
      Serial.print("Humidity: ");Serial.println(humidity);
      Serial.print("Temperature: ");Serial.println(tempc);
      // Serial.print("Dewpoint: ");Serial.println(dewpoint);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Error\n\r\tChecksum error");
      break;
    case DHTLIB_ERROR_ISR_TIMEOUT:
      Serial.println("Error\n\r\tISR time out error");
      break;
    case DHTLIB_ERROR_RESPONSE_TIMEOUT:
      Serial.println("Error\n\r\tResponse time out error");
      break;
    case DHTLIB_ERROR_DATA_TIMEOUT:
      Serial.println("Error\n\r\tData time out error");
      break;
    case DHTLIB_ERROR_ACQUIRING:
      Serial.println("Error\n\r\tAcquiring");
      break;
    case DHTLIB_ERROR_DELTA:
      Serial.println("Error\n\r\tDelta time too small");
      break;
    case DHTLIB_ERROR_NOTSTARTED:
      Serial.println("Error\n\r\tNot started");
      break;
    default:
      Serial.println("Unknown error");
  }

  //Measure the Barometer temperature in C from the MPL3115A2
  baroTemp = baro.getTemperature();

  //Measure Pressure from the MPL3115A2
  pascals = baro.getPressure();

  //Calc winddir
  winddir = get_wind_direction();
  winddiravg[seconds_2m] = winddir;

  //Calc windspeed
  windspeedkmh = get_wind_speed();
  windspdavg[seconds_2m] = windspeedkmh;

  //Check to see if this is a gust for the minute
  if (windspeedkmh > windgust_10m[minutes_10m]) 
  {
    windgust_10m[minutes_10m] = windspeedkmh;
    windgustdirection_10m[minutes_10m] = winddir;
  }

  //Check to see if this is a gust for the day
  if (windspeedkmh > windgustkmh)
  {
    windgustkmh = windspeedkmh;
    windgustdir = winddir;
  }

  //Calc windspdkmh_avg2m
  double tempo = 0;
  for (int i = 0; i < 120; i++)
    tempo += windspdavg[i];
  tempo /= 120.0;
  windspdkmh_avg2m = tempo;

  //Calc winddir_avg2m
  tempo = 0; //Can't use winddir_avg2m because it's an int
  for (int i = 0; i < 120; i++)
    tempo += winddiravg[i];
  tempo /= 120;
  winddir_avg2m = tempo;

  //Calc windgustkmh_10m
  //Calc windgustdir_10m
  //Find the largest windgust in the last 10 minutes
  windgustkmh_10m = 0;
  windgustdir_10m = 0;
  //Step through the 10 minutes
  for (int i = 0; i < 10; i++)
  {
    if (windgust_10m[i] > windgustkmh_10m)
    {
      windgustkmh_10m = windgust_10m[i];
      windgustdir_10m = windgustdirection_10m[i];
    }
  }

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainmm = 0;
  for (int i = 0; i < 60; i++)
    rainmm += rainHour[i];
}

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge
{
  raintime = millis();                // grab current time
  raininterval = raintime - rainlast; //  calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainmm += 0.2794;       //Each dump is 0.2794mm of water
    rainHour[minutes] += 0.2794; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation)
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (228KMH max reading) after the reed switch closes
  {
    lastWindIRQ = millis();
    windClicks++;           
  }
}

void printInfo()
{
  //This function prints the weather data out to the default Serial Port
  Serial.print("Wind_Dir:");
  switch (winddir)
  {
  case 0:
    Serial.print("North");
    break;
  case 1:
    Serial.print("NE");
    break;
  case 2:
    Serial.print("East");
    break;
  case 3:
    Serial.print("SE");
    break;
  case 4:
    Serial.print("South");
    break;
  case 5:
    Serial.print("SW");
    break;
  case 6:
    Serial.print("West");
    break;
  case 7:
    Serial.print("NW");
    break;
  default:
    Serial.print("No Wind");
    // if nothing else matches, do the
    // default (which is optional)
  }

  Serial.print(" Wind_Speed:");
  Serial.print(windspeedkmh, 1);
  Serial.print("mph, ");

  Serial.print("Rain:");
  Serial.print(rainmm, 2);
  Serial.print("mm, ");

  Serial.print("Temp:");
  Serial.print(tempc);
  Serial.print("C, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("%, ");

  Serial.print("Baro_Temp:");
  Serial.print(baroTemp);
  Serial.print("C, ");

  Serial.print("Pressure:");
  Serial.print(pascals / 100);
  Serial.print("hPa, ");
  Serial.print((pascals / 100) * 0.0295300);
  Serial.println("in.Hg");
  //The MPL3115A2 outputs the pressure in Pascals. However, most weather stations
  //report pressure in hectopascals or millibars. Divide by 100 to get a reading
  //more closely resembling what online weather reports may say in hPa or mb.
  //Another common unit for pressure is Inches of Mercury (in.Hg). To convert
  //from mb to in.Hg, use the following formula. P(inHg) = 0.0295300 * P(mb)
  //More info on conversion can be found here:
  //www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf

  //If in altitude mode, print with these lines
  //Serial.print("Altitude:");
  //Serial.print(altf);
  //Serial.println("ft.");
}

int resetme(String args) {
  System.reset();
  return 0;
}