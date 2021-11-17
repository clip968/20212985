// Arduino pin assignment
#include <Servo.h>
#define PIN_LED 9
#define PIN_IR A0
#define PIN_SERVO 10

Servo myservo;
// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 70 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 1000// maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.3 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.
#define dist_mid 300

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion


void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  
// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = ir_distance();
  dist_ema = (alpha*dist_raw) + ((1-alpha)*dist_ema);

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("ema:");
  Serial.print(dist_ema);
//  Serial.print(map(dist_ema,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

// turn on the LED if the distance is between dist_min and dist_max
  if(dist_ema >= dist_mid) {
    myservo.writeMicroseconds(1240);
  }
  else {
    myservo.writeMicroseconds(1680);
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
