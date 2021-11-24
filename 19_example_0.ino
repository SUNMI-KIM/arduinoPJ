// Arduino pin assignment
#include <Servo.h>
#define PIN_LED 9
#define PIN_IR A0
#define PIN_SERVO 10
Servo myservo;
int a, b; // unit: mm

static long apt = 0; 
#define interval 2
unsigned long oldmil, time_curr;
int fc = 10;
float dt = interval/1000.0;
float lambda = 2*PI*fc*dt;
float calidist = 0.0, filter = 0.0, prev = 0.0;
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1370 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.0

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _KP 3.0
#define _Ki 0.3
#define _Kd 0.3

float dist_target;
float dist_min, dist_max, dist_raw, dist_ema, dist_prev;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  time_curr = millis();
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_ema = 0.0;
  float alpha = _DIST_ALPHA;
  duty_chg_per_interval = ((_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_DIST / 1000));

  
// initialize serial port
  Serial.begin(57600);

  a = 125; //70;
  b = 425; //300;
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1370);
  
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val - 20;
}

void loop() {
   /*if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
  last_sampling_time_dist += _INTERVAL_DIST;
  event_dist = true;
    }
    if (time_curr >= last_sampling_time_dist + _INTERVAL_SERVO){
  last_sampling_time_servo += _INTERVAL_SERVO;
  event_servo = true;
    }
    if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
  last_sampling_time_serial += _INTERVAL_SERIAL;
  event_serial = true;*/

  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  unsigned long dmil = 0;
  unsigned long mil = millis();
  if (mil != oldmil) { 
  dmil = mil-oldmil; 
  oldmil = mil;   
} 
  apt -= dmil; 
  if (apt <= 0) {  
  apt += interval; 
  calidist = dist_cali;
  filter = lambda/(1+lambda)*calidist+1/(1+lambda)*prev ; 
  prev = filter;
  
  /*if(event_dist) {
      event_dist = false;//[2981] 이벤트가 실행되었으므로 재실행을 위한 값 초기화*/

  // PID control logic
     error_curr= _DIST_TARGET - filter; //[3000] 에러는 목표거리에서 탁구공의 거리를 뺀값
     pterm = _KP*error_curr; //[3000] 제어식에서의 비례항
     iterm =  _Kd * (error_curr - error_prev) / time_curr; //[3000] 제어식의 미분항
     dterm += _Ki * error_curr * time_curr; //[3000] 제어식에서의 적분항

     control = pterm + iterm + dterm; //[3000] PID결과값
     error_prev = error_curr; //[3000] 이전의 에러값을 저장
     // duty_target = f(duty_neutral, control)

      duty_target = map(error_curr, 100, -100, 1510, 1230);
     
   }
  
  //if(event_servo) {
      //event_servo = false; //[1352]server Event handler가 false
      // adjust duty_curr toward duty_target by duty_chg_per_interval
      // update servo position       
      myservo.writeMicroseconds(duty_target); 
      //[3000] 서보위치 업데이트 error_curr 뒤의 숫자는 수정가능 
  //}
  
  //if(event_serial) {
     //event_serial = false; //[2989]  이벤트 실행 후 초기화
     Serial.println("Min:0,Low:200,dist:"); //[2983] “Min:0,Low:200,dist:” 문구 출력
     Serial.println(dist_raw); //[2979] dist_raw 값(적외선 센서에서 돌려받은 값) 출력
     Serial.println(",pterm:"); 
     Serial.println(error_curr);
     /*Serial.println(",duty_target:"); //[2994] duty_target 값 출력
     Serial.println(duty_target); //[2994] duty_target 값 출력*/
     Serial.println(",filter:"); // [2980] “duty_curr” 출력
     Serial.println(filter); // [2980] duty_curr 값 출력
             
  //}


 
  delay(20);
//}
    //}
}
