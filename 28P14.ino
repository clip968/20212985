#include <Servo.h>
#include <PID_v1.h>
#include <math.h>
/////////////////////////////
// Configurable parameters //
/////////////////////////////
#define interval 3 // milliseconds
// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10 // [1352] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0 // [1352] IR센서를 아두이노 A0 핀에 연결


// Framework setting
#define _DIST_TARGET 255   // [1352]목표값을 탁구공 중심 위치까지 거리 255mm로 Fix
#define _DIST_MIN 60 // [1352] 최소 측정 거리를 100mm로 설정
#define _DIST_MAX 800 // [1352] 측정 거리의 최댓값을 410mm로 설정


// Distance sensor
#define _DIST_ALPHA 0.075// [2979] 알파값 설정

// Servo range
//#define _DUTY_MIN 1350    // //[2998] Servo Minimum microseconds
//#define _DUTY_NEU 1650   //[2983] Servo 중간값 설정
//#define _DUTY_MAX 1950       // [2979] Servo Max값 설정
#define _DUTY_MIN 1480 
#define _DUTY_NEU 1780
#define _DUTY_MAX 2080

// Servo speed control
#define _SERVO_ANGLE 50 // [2992] 서보 각 설정
#define _SERVO_SPEED 800 // [2976] 서보 스피드 설정

// Event periods
#define _INTERVAL_DIST 10   // [2987] 거리측정 INTERVAL값 설정
#define _INTERVAL_SERVO 22  // [2980] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  //[2989] 시리얼 출력 INTERVAL 설정
#define DIST_10C 60
#define DIST_40C 350

#define INTERVAL 24  // servo update interval
#define _ITERM_MAX 40
#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100
// PID parameters
float Kp = 2.15;                //P게인 값
float Ki = 0.3;                //I게인 값
float Kd = 65.95;                  //D게인 값  

//////////////////////
// global variables //
//////////////////////
static long apt = 0; 

unsigned long oldmil;

double Time = 0.3 ;
int fc =3; 
float dt = interval/1000.0; 
float lambda = 2*PI*fc*dt;
float calidist = 0.0, filter = 0.0, prev = 0.0;
// Servo instance
Servo myservo;  //[2991] create servo object to control a servo
double controls = 0;
// Distance sensor
float dist_target; // location to send the ball [2976] 공 위치
float dist_raw, dist_ema, dist_cali; // [2981] 거리 변수 설정(현재, ema 필터 적용, 직전값) 
float alpha;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; //[2989] 논리형 변수 설정
double Setpoint, Input, Output, ServoOutput;
unsigned long currentTime, previous_time; 
unsigned long last_sampling_time;


double last_error;
// Servo speed control
int duty_chg_per_interval; //[1352] 주기동안 duty 변화량 변수 
int duty_target, duty_curr;//[1352] 서보모터의 목표위치, 서보에 실제 입력할 위치
int duty_chg_max;
int duty_chg_adjust;
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

int a = 83;
int b = 403;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  
// move servo to neutral position
  myservo.attach(PIN_SERVO); //[2998] Servo Attaching 
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);//[2998] Servo Set to neutral position


  // initialize global variables
  duty_curr=_DUTY_MIN;  //[2999] dist_min 값 적용
  dist_target=_DIST_TARGET; 
  alpha = _DIST_ALPHA; //[2999] alpha 선언 및 값 적용
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  
  
  // initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_DIST / 1000); //[2985] duty_chg_per_interval 설정
  //duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  //duty_chg_adjust = (float) duty_chg_max * INTERVAL / _RAMPUP_TIME;
 // initial speed is set to 0.
  //duty_chg_per_interval = 0;
  pause_time = 0.55;
  //toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  //toggle_interval = 1; // to demonstrate overshoot
  //toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
   
}
  


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
 
  unsigned long time_curr = millis();
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
          last_sampling_time_dist += _INTERVAL_DIST;
          event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
          last_sampling_time_servo += _INTERVAL_SERVO;
          event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
          last_sampling_time_serial += _INTERVAL_SERIAL;
          event_serial = true;
  }//[2981] 인터벌 값과 샘플링 차이의 합과 비교해서 각각의 이벤트 실행
  //[2998] Fixed some indentation, tabs and more
  
  
  ////////////////////
  // Event handlers //
  ////////////////////
  
  if(event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
    dist_ema = ir_distance_filtered(); //[3000]raw_dist 거리값 설정
    //dist_cali = 100 + 300.0 / (b - a) *(dist_raw - a);
    
    //if (dist_ema == 0){                  
    //    dist_ema = dist_raw;               
    //  }                                    
    //  else{
    //    dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;   
    //  }  
    
  // PID control logic
 
    error_curr = dist_target - dist_ema; //[3000] 에러는 목표거리에서 탁구공의 거리를 뺀값
    iterm += Ki * (error_curr);
    dterm = Kd * (error_curr-last_error)/Time;
    pterm = error_curr * Kp; //[3000] 제어식에서의 비례항
    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    
    control = dterm + pterm + iterm;  //[3000] PID결과값  
    
    
    
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    last_error = error_curr;
  
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  // [1352] 서보의 가동범위를 고정
    if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}//[1352] 조건일때 duty_target값을 _DUTY_MAX으로
    else if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}//[1352] 조건일때 duty_target값을 _DUTY_MIN으로
    
    
  }
    
  if(event_servo) {
    event_servo=false;
  
     // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
         if(duty_curr > duty_target) duty_curr = duty_target;
      }
    else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
      }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr); 
  }

  

  
  if(event_serial) {
      event_serial = false; //[2989]  이벤트 실행 후 초기화
     Serial.print("dist_ir:"); //[2983] “Min:0,Low:200,dist:” 문구 출력
     Serial.print(dist_ema);
     Serial.print(",T:");
     Serial.print(dist_target);
     Serial.print(",P:"); 
     Serial.print(map(pterm, -1000, 1000, 510, 610));
     Serial.print(",D:"); 
     Serial.print(map(dterm, -1000, 1000, 510, 610));
     Serial.print(",I:"); //[2983] “Min:0,Low:200,dist:” 문구 출력
     Serial.print(map(iterm, -1000, 1000, 510, 610));
     Serial.print(",DTT:"); //[2994] duty_target 값 출력
     Serial.print(map(duty_target, 1000, 2000, 410, 510)); //[2994] duty_target 값 출력
     Serial.print(",DTC:"); // [2980] “duty_curr” 출력
     Serial.print(map(duty_curr, 1000, 2000, 410, 510)); // [2980] duty_curr 값 출력
     Serial.println(",-G:245,+G:265,m:0,M:800");
     
  


    
  
  }

// update last sampling time
 
  }

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  float dist =ir_distance(); //[2998] calling distance from ir_distance()
   unsigned long dmil = 0; //[2998] return previous distance
   unsigned long mil = millis();  //[2998] get current time
  
   if (mil != oldmil) { //[2998] set time difference on dmil for checking previous time
     dmil = mil-oldmil; 
     oldmil = mil;   
   } 
  
   apt -= dmil; //[2998] set difference for dmil and apt time
  
   if (apt <= 0) {  //[2998] check time before calculating filtered time
     apt += interval;  
     filter = lambda / (1 + lambda) * dist + 1 / (1 + lambda) * prev; 
     //[2998] Calculate filtered distance  
     prev = filter; //[2998] Sensor filter previous  value update
   }
   float calidist = 100.0 + 300.0 / (DIST_40C - DIST_10C) * (filter - DIST_10C);
   return calidist;
   // for now, just use ir_distance() without noise filter.
}
