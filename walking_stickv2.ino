/*          SMART ULTRASONIC RANGE SENSOR WALKING STICK WITH DISTRESS SIGNAL AND
 *           COMPANION MOBILE APPLICATION v2 (Arduino Side)
 * 
 * Design project by:
 * Alaba, Jerryco G.
 * Buebo, Reiner Carlo
 * Erracho, Buena Mar B.
 * Nunez, Almira Faye
 * 
 */

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <NewPing.h>
#include <math.h>
#define WATER_THRESHOLD 100
#define MAXMOTOR 120// maximum analogwrite for motor
#define MINMOTOR 30
#define MAXDIST 300 // maximum distance to read
#define btn_alert A1 // digital pin for alert button
#define mode A2 // indoor or outdoor mode switch selection
#define OK 0
#define GPS_ERROR 1
#define SMSERROR 2
#define stick_range 55
#define WATER_SENSOR A3
#define OFFSET 10
typedef struct gpsobject // contains the current coordinate of the stick and time
{
  unsigned long date = TinyGPS::GPS_INVALID_DATE;
  unsigned long timef = TinyGPS::GPS_INVALID_TIME;
  float flat = TinyGPS::GPS_INVALID_F_ANGLE;
  float flon = TinyGPS::GPS_INVALID_F_ANGLE;
};

TinyGPS gps;  // gps parser object

SoftwareSerial GPSserial(12, 13);  // serial comm object for gps
SoftwareSerial GSMserial(10, 9);  // serial comm object for gsm/gprs

gpsobject GPSobj; 

char* const recipient = "639507065295";  //message recipient
const char request[] = "{123456,2}";
String const me = "123456";  // stick identity
const int debounce_time = 50; // ms of acceptable debounce time
const int gps_tries = 5;
String default_msg = "GPS Error";

//sonar objects (L, M, R, T)
NewPing sonarm[4] = {NewPing (8, 8, MAXDIST), NewPing (7, 7, MAXDIST), NewPing (2, 2, MAXDIST), NewPing (4, 4, MAXDIST)};

// motor pins (L, M, R, T)
int motor[] = {5, 11, 6, 3}; // left, middle, right motors

// function prototypes
int sendAlert(int a);  // sends an alert to the designated number and return a status code
bool checkpress(int pin, int dur);  // check if a pin was high for a duration of time (debounce handled)
int updategps(void);
void off();
int sendSMS(String m);
bool equalto(char a[]);

void setup() 
{
  delay(4000);
  //pinMode(3, OUTPUT);
  pinMode(btn_alert, INPUT);
  pinMode(mode, INPUT);
  for (int i = 0; i < 4; i++)
    pinMode(motor[i], OUTPUT);  
  Serial.begin(9600);
  
  //begin software serial
  GPSserial.begin(9600); 
  GSMserial.begin(9600);
  GSMserial.listen();
  delay(1000);

  //Set SMS format to ASCII
  GSMserial.write("AT+CMGF=1\r\n");
  delay(1700);
  GSMserial.write("AT+CNMI=1,2,0,0,0\r\n");
  delay(1000);

  Serial.println("Setup Complete");
}

void loop() 
{
  if (1)  // on/off switch for the stick
  {
    // get current mode.
    // TODO:  get current mode distance
    int mdist = digitalRead(mode) ? 200 : 100; // 2m(true) or 1m(false) detection range.
    
    for (int i = 0; i < 4; i++) // loop through every sensor
    {
      float MotorOut = 0;
      int measure = sonarm[i].ping_cm();
      

        int dist = measure - stick_range; // subtract the detection range of stick
        dist = dist > mdist ? mdist : dist;
        if (measure > stick_range)
        {
          float a = 1/pow(mdist, 2);
          float y  = a * pow(dist - mdist, 2);  // use quadratic function to produce pwm strength of the motor.
          float towrite = ((MAXMOTOR - MINMOTOR) * y) + MINMOTOR;
          MotorOut = towrite;
          
        }
        // offset
        else if (measure > OFFSET){
          MotorOut = MAXMOTOR -30;
        }
        
      
      
      
      analogWrite(motor[i], MotorOut);
    }
    
      
  }
  
  // check water sensor for
  bool state = true;
  /*while (analogRead(WATER_SENSOR) > WATER_THRESHOLD){
    if (state){
      onMax();
    }
    else{
      off();
    }
    state = !state;
    delay(300);
  }*/
   
  // wait for alert button press
  if (digitalRead(btn_alert))
  {
    Serial.println("pressed");
    // TODO: off all vibrations
    off();
    
    // wait one whole second of press
    if (checkpress(btn_alert, 3000))
    {
      GPSserial.listen();
      sendAlert(1);
      GSMserial.listen();
    }
  }

  // check for message
  if (GSMserial.available())  // flush serial buffer
  {
    Serial.println("msg recieved");
    off();
    int index = 0;
    char abuffer[11];
    bool enable = false;
    for (double i = millis(); 3000 > millis() - i && index < 10;)
    {
      if (GSMserial.available())
      {
        abuffer[index] = GSMserial.read();
        Serial.write(abuffer[index]);
        if (abuffer[index] == '{')
        {
          enable = true;
        }
        if (enable)
          index++;
      }
    }
    if(index > 0)
    {
      // terminate buffer
      abuffer[index] = '\0';
      if (equalto(abuffer))
      {
        Serial.println("ok");
        sendAlert(2);
      }
    }
    
  }
  // clear buffer
  while (GSMserial.available())
    GSMserial.read();
  
    delay(20);
}

int updategps(void)
{
  int code = GPS_ERROR;
  delay(500);
  bool newData = false;

  // obtain valid gps readings (limited tries)
  int i;
  for (i = 0; i < gps_tries; i++)
  {
    Serial.println(i);
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (GPSserial.available())
      {
        char c = GPSserial.read();
        if (gps.encode(c))
          newData = true;
      }
    }

    if (newData)  //validate data obtained
    {
      newData = false;
      gps.f_get_position(&GPSobj.flat, &GPSobj.flon);
      gps.get_datetime (&GPSobj.date, &GPSobj.timef);
      
      if (GPSobj.flat != TinyGPS::GPS_INVALID_F_ANGLE && GPSobj.flon !=  TinyGPS::GPS_INVALID_F_ANGLE // error GPS reading
      && GPSobj.date != TinyGPS::GPS_INVALID_DATE && GPSobj.timef != TinyGPS::GPS_INVALID_TIME)
      {
        /*Serial.println(GPSobj.flat, 6);
        Serial.println(GPSobj.flon, 6);
        Serial.println(GPSobj.date, 6);
        Serial.println(GPSobj.timef, 6);*/
        return OK;
      }
      else
      {
        return GPS_ERROR;
      }
    }
  }
  if (i == gps_tries)
  {
    return GPS_ERROR;
  }
}


int sendAlert(int a)
{ 
  String message = "";
  if (updategps() == GPS_ERROR) // debug equals
    message = default_msg;

  else{
    String message = "{'a':" + me + ",'b':"+a+",'c':" + (GPSobj.flat * 100000) + ",'d':" + (GPSobj.flon * 100000) + ",'e':\"" + GPSobj.timef +"\"}";
  }
  GSMserial.listen();
  /*String message = "{'a':" + me + ",'b':"+a+",'c':" + "1449080" + ",'d':" + "12105930" + ",'e':\"" + GPSobj.timef +"\"}";
  GSMserial.listen();*/ 
  return sendSMS(message);  
}

bool checkpress(int pin, int dur)
{
  unsigned long start = millis();
  while (millis() - start < dur )
  {
    if (!digitalRead(pin))  // button was off
    {
      unsigned long off = millis();
      while (!digitalRead(pin))
      {
        if (millis() - off > debounce_time)
          return false;
      }
    }
  }
  return true;
}

void off() // off all vibrations
{
  for(int i = 0; i<4; i++)
  {
    analogWrite(motor[i], 0);
  }
}

void onMax(){
  for (int i = 0; i < 4; i++){
    analogWrite(motor[i], MAXMOTOR);
  }
}


int sendSMS(String m)
{
  int len = m.length() + 1;
  char message[len];
  m.toCharArray(message, len);
  Serial.println(message);
  //Set SMS format to ASCII
  GSMserial.write("AT+CMGF=1\r\n");
  delay(1000);
  //Send new SMS command and message number
  GSMserial.write("AT+CMGS=\"");
  GSMserial.write(recipient);
  GSMserial.write("\"\r\n");
  delay(1000);
   
  //Send SMS content
  GSMserial.write(message);
  delay(1000);
   
  //Send Ctrl+Z / ESC to denote SMS message is complete
  GSMserial.write((char)26);
  delay(1000);
  for (unsigned long start = millis(); millis()-start < 6000;)
  { 
    char prev;
    if(GSMserial.available())
    {
      char curr = GSMserial.read();
      if (prev == 'O' && curr == 'K') // message was sent
      {
        Serial.println("OK");
        for (int i = 0; i < 2; i++)
        {
          for (int i = 0; i<3; i++)
            analogWrite(motor[i], 60);

          delay(800);
          off();
          delay(800);
        }
        return OK;
      }
      prev = curr;
    }
  }
  GSMserial.flush();
  // message was not sent
  return SMSERROR;
}

bool equalto(char a[])
{
  for (int i = 0; i < 10; i++)
  {
    if (a[i] != request[i])
      return false;
  }
  return true;
}

