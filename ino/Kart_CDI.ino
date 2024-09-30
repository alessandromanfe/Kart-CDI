#include "esp_timer.h"
//#include <Servo.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <can_regdef.h>

#define FS sizeof(double)
#define SPARK_PIN 19 //21
#define PICKUP_PIN 22 //22
#define ENABLE_PIN 18 //32
#define SERVO_PIN 33
#define RELAY_PIN 23
#define START_PIN 16
#define STOP_PIN 17

#define REV_LIMITER 13500
#define CTS_MIN 7500
#define CTS_MAX 8500

#define CAN_UPDATE_FREQ 30 //Hz
#define RPM_BUF_SIZE 5
#define STATIC_TIMING 20 //Gradi anticipo fisso (rpm < 1000)
#define PICKUP_ANGLE 46

#define CONVERSION_FACTOR 166666.666667

constexpr double canDelay = 1.0e6 / CAN_UPDATE_FREQ;
int64_t prevCanTimer = 0;
float rpmBuf[RPM_BUF_SIZE] = {0};
int rpmIndex = 0;

CAN_device_t CAN_cfg;
CAN_frame_t tx_frame;

//Servo CTS;
//int CTS_angle = 0;
volatile bool debug = false;
volatile bool spark = false;
volatile bool isRunning = false;
bool isCranking = false;
volatile bool isStopping = false;
bool isKilling = false;

volatile int64_t time_since_boot = 0;
volatile int64_t period = 0;
volatile int size = 0;
volatile int64_t angle_delay;
int64_t sangle_delay;

int64_t killingTimer = 0;
volatile int64_t previousTimer = 0;
volatile int64_t previousPeriod = 0;
volatile double phase = 0;
esp_timer_handle_t spark_timer;
esp_timer_handle_t enable_timer;

volatile unsigned char rpm2 = 0;

volatile double rpm = 0;
volatile int count = 0;
double current_rpm = 0;
volatile double current_period = 0;

double srpm;


int mapnum = 1;

volatile double multiplier = 2.0;

volatile double map1[][2] = {
  {1000.0,20.0},
  {3000.0,20.0},
  {8000.0,20.0},
  {10000.0,18.0},
  {11000.0,10.0},
  {12000.0,7.0}
};
int map1_size = 6;

/*
int64_t delayFromRPM(float rpm, int size){
  if(rpm<map1[0][0]) return CONVERSION_FACTOR/rpm*STATIC_TIMING;
  if(rpm>=map1[size-1][0]) return CONVERSION_FACTOR/rpm*map1[size-1][1];
  for(int i=1; i<size; i++){
    if(rpm<map1[i][0]){
      phase = (map1[i][1]-map1[i-1][1])/(map1[i][0]-map1[i-1][0])*(rpm-map1[i-1][0]) + map1[i-1][1];
      break;
    }
  }
  return (int64_t)(CONVERSION_FACTOR/rpm*phase);
}
*/

void IRAM_ATTR pickup_signal(){

  debug = true;
  time_since_boot = esp_timer_get_time();
  period = time_since_boot - previousTimer;
  //if (period < previousPeriod/2) return;
  esp_timer_stop(spark_timer);
  esp_timer_stop(enable_timer);
  previousTimer = time_since_boot;
  previousPeriod = period;
  //current_period = period;
  if(isStopping) return;
  /*if(period>=1000000){
    //digitalWrite(ENABLE_PIN, LOW);
    isRunning = false;
    count = 0;
    return;
  }*/
  if(!isRunning && count < 3){
    count++;
    return;
  }
  spark = true;
  isRunning = true;
  //volatile double rpm = 60000000.0/period;
  rpm = (60.0 / ((double)(period))) * 1000000.0;
  rpm2 = round(rpm/100.0);
  if(rpm>REV_LIMITER) return;

  size = map1_size;
  if(rpm<map1[0][0]) angle_delay = CONVERSION_FACTOR/rpm*(PICKUP_ANGLE - STATIC_TIMING);
  else if(rpm>=map1[size-1][0]) angle_delay = CONVERSION_FACTOR/rpm*(PICKUP_ANGLE - map1[size-1][1]);
  else{
    for(int i=1; i<size; i++){
      if(rpm<map1[i][0]){
        phase = (map1[i][1]-map1[i-1][1])/(map1[i][0]-map1[i-1][0])*(rpm-map1[i-1][0]) + map1[i-1][1];
        angle_delay = (int64_t)(CONVERSION_FACTOR/rpm*(PICKUP_ANGLE - phase));
        break;
      }
    }
  }
  ESP_ERROR_CHECK(esp_timer_start_once(spark_timer, angle_delay));//getDelay(period)));
  ESP_ERROR_CHECK(esp_timer_start_once(enable_timer, 2000 + angle_delay));
}

static void spark_timer_callback(void* arg){
  digitalWrite(ENABLE_PIN, LOW);
  ets_delay_us(10);
  digitalWrite(SPARK_PIN, HIGH);
  ets_delay_us(30);
  digitalWrite(SPARK_PIN, LOW);
  //ets_delay_us(2000);
  //digitalWrite(ENABLE_PIN, HIGH);
}

static void enable_timer_callback(void* arg){
  digitalWrite(ENABLE_PIN, HIGH);
}


const esp_timer_create_args_t spark_timer_args = {
  .callback = &spark_timer_callback,
  .name = "spark_timer"
};

const esp_timer_create_args_t enable_timer_args = {
  .callback = &enable_timer_callback,
  .name = "enable_timer"
};


void sendCan(){
  float rpmx = 0;
  for(int i=0; i<RPM_BUF_SIZE; i++) rpmx += rpmBuf[i];
  uint16_t rpmi = round(rpmx/RPM_BUF_SIZE);

  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x300;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u64 = 0;
  tx_frame.data.u8[0] = (unsigned char)(rpmi >> 8);
  tx_frame.data.u8[1] = (unsigned char)(rpmi & 0xff);
  ESP32Can.CANWriteFrame(&tx_frame);
}


void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PICKUP_PIN, INPUT_PULLUP);
  pinMode(SPARK_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);

  attachInterrupt(PICKUP_PIN, pickup_signal, FALLING);
  ESP_ERROR_CHECK(esp_timer_create(&spark_timer_args, &spark_timer));
  ESP_ERROR_CHECK(esp_timer_create(&enable_timer_args, &enable_timer));
  digitalWrite(SPARK_PIN, LOW);
  digitalWrite(ENABLE_PIN,LOW);
  digitalWrite(RELAY_PIN,LOW);
  //digitalWrite(ENABLE_PIN, HIGH);

  //CTS.attach(SERVO_PIN);
  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_25;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
  ESP32Can.CANInit();

}

void loop() {
  /*
  if(debug){
    Serial.println("Pickup");
    debug = false;
  } 
  */

  /*
  if(!isCranking && !isRunning && digitalRead(START_PIN)==LOW){
    isStopping = false;
    isCranking = true;
    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(RELAY_PIN, HIGH);
  }
  else if(digitalRead(START_PIN)==HIGH){
    isCranking = false;
    digitalWrite(RELAY_PIN, LOW);
  }
  /*
  /*
  if(!isKilling && digitalRead(STOP_PIN)==LOW){
    killingTimer = esp_timer_get_time();
    isKilling = true;
  }
  if(digitalRead(STOP_PIN)==LOW && esp_timer_get_time()-killingTimer>100000){
    Serial.println("STOP");
    isStopping = true;
    isRunning = false;
    digitalWrite(ENABLE_PIN,LOW);
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(SPARK_PIN,LOW);
  }
  if(digitalRead(STOP_PIN)==HIGH){
    isKilling = false;
  }
  */
  /*
  if(digitalRead(STOP_PIN)==LOW){
    isStopping = true;
    isRunning = false;
    digitalWrite(ENABLE_PIN,LOW);
    digitalWrite(RELAY_PIN, LOW);
  }
  */

  if(esp_timer_get_time()-prevCanTimer>canDelay){
    prevCanTimer = esp_timer_get_time();
    sendCan();
  } 

  if(esp_timer_get_time()-previousTimer>1000000)
  {
    count = 0;
    isRunning = false;
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(SPARK_PIN,LOW);
  }

  //if(!isRunning && !isCranking){
  //  digitalWrite(ENABLE_PIN,LOW);
  //}

  //CTS
  /*
  if(rpm > CTS_MIN){
    CTS_angle = (rpm-CTS_MIN) / (CTS_MAX-CTS_MIN) * 180;
    CTS.write(min(CTS_angle,180));
  }
  else if(CTS_angle != 0){
    CTS_angle = 0;
    CTS.write(CTS_angle);
  }
  */
  
  if(spark){
    //Serial.println("spark");
    /*
    noInterrupts();
    current_rpm = rpm*1.0;
    interrupts();
    */
    noInterrupts();
    srpm = rpm;
    sangle_delay = angle_delay;
    interrupts();

    if(srpm < 20000){
      rpmBuf[rpmIndex] = srpm;
      rpmIndex = (rpmIndex+1)%RPM_BUF_SIZE;
    }
    Serial.print(srpm);
    Serial.print("\t");
    //Serial.print(rpm2*100);
    //Serial.print("\t");
    Serial.print(phase);
    Serial.print("\t");
    Serial.println(sangle_delay);
    //Serial.println(current_period);
    spark=false;
  }
  
  /*
  Serial.print(rpm);
  Serial.print("\t");
  Serial.println(phase);
  */

}
