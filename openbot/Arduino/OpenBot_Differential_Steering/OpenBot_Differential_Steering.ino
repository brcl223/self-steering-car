#define DIY 0
#define PCB_V1 1
#define PCB_V2 2
//#include <PinChangeInterrupt.h>
#define OPENBOT PCB_V2

#if (OPENBOT == DIY)
#define PIN_PWM_L1 5
#define PIN_PWM_L2 6
#define PIN_PWM_R1 9
#define PIN_PWM_R2 10
#define PIN_SPEED_L 2
#define PIN_SPEED_R 3
#define PIN_VIN A7
#define PIN_TRIGGER 12
#define PIN_ECHO 11
#define PIN_LED_LB 4
#define PIN_LED_RB 7
#elif (OPENBOT == PCB_V1)
#define PIN_PWM_L1 9
#define PIN_PWM_L2 10
#define PIN_PWM_R1 5
#define PIN_PWM_R2 6
#define PIN_SPEED_L 2
#define PIN_SPEED_R 4
#define PIN_VIN A7
#define PIN_TRIGGER 3
#define PIN_ECHO 3
#define PIN_LED_LB 7
#define PIN_LED_RB 8
#elif (OPENBOT == PCB_V2)
#define PIN_PWM_L1 9
#define PIN_PWM_L2 10
#define PIN_PWM_R1 5
#define PIN_PWM_R2 6
#define PIN_SPEED_L 2
#define PIN_SPEED_R 3
#define PIN_VIN A7
#define PIN_TRIGGER 4
#define PIN_ECHO 4
#define PIN_LED_LB 7
#define PIN_LED_RB 8
#endif
char input;
#define WAITTIME 0.1
//#define SLEEP_FOR(sec) for(int __i = 0; __i < (sec); __i++) delay(1000)
#define SLEEP_FOR(sec) (void)0
#define SPEED 105
#define HIGH_SPEED 200
#define LOW_SPEED 100

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_PWM_L1, OUTPUT);
  pinMode(PIN_PWM_L2, OUTPUT);
  pinMode(PIN_PWM_R1, OUTPUT);
  pinMode(PIN_PWM_R2, OUTPUT);
  pinMode(PIN_LED_LB, OUTPUT);
  pinMode(PIN_LED_RB, OUTPUT);

}

void loop()
{
  
  
  if (Serial.available()) {
    input = Serial.read();
    Serial.println(input);

    if (input == 'R') {
      analogWrite(PIN_PWM_L1, HIGH_SPEED);
      analogWrite(PIN_PWM_L2, 0);
      analogWrite(PIN_PWM_R1, LOW_SPEED);
      analogWrite(PIN_PWM_R2, 0);
     
      SLEEP_FOR(WAITTIME);
    }
    else if (input == 'F'){
      analogWrite(PIN_PWM_L1, SPEED);
      analogWrite(PIN_PWM_L2, 0);
      analogWrite(PIN_PWM_R1, SPEED);
      analogWrite(PIN_PWM_R2, 0);
      SLEEP_FOR(WAITTIME);
    }
    else if (input == 'L') {
      analogWrite(PIN_PWM_L1, LOW_SPEED);
      analogWrite(PIN_PWM_L2, 0);
      analogWrite(PIN_PWM_R1, HIGH_SPEED);
      analogWrite(PIN_PWM_R2, 0);
      SLEEP_FOR(WAITTIME);
    }
    
    //else{
    //  analogWrite(PIN_PWM_L1, SPEED);
    //  analogWrite(PIN_PWM_L2, 0);
    //  analogWrite(PIN_PWM_R1, 0);
    //  analogWrite(PIN_PWM_R2, 0);
    //  SLEEP_FOR(WAITTIME);
    //}
    
  }


}
