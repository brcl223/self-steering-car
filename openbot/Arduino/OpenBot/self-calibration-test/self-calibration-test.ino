/*
 * OPEN BOT SPECIFIC CODE
 */
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






/*
 * CAR SPECIFIC HEADER HERE
 */

void turn_left(int mag);
void turn_right(int mag);
void stop_car();
void forward(int mag);
void reverse(int mag);
void differential(int v1, int v2);







/*
 * COMMAND PARSING SPECIFIC HEADER HERE
 */

#define READBUFLEN 30
#define MINBUFLEN 7
char readbuf[READBUFLEN];
enum class Motion { Stop, Forward, Backward, Left, Right, Differential, Error };
typedef struct {
  Motion motion;
  int v1;
  int v2;
  size_t dur;
} Cmd;

byte readline();
Motion parse_motion(char mo);
Cmd parse_cmd(char *buf, size_t buflen);
void debug_print_cmd(Cmd &cmd);
byte i;

// global variables
Cmd cmd;
byte readsize;
unsigned long startTime;
unsigned long currentTime;
unsigned long timeDifference;

/*
 * MAIN LOOP CODE HERE
 */
 
void setup()
{
  Serial.begin(9600);
  pinMode(PIN_PWM_L1, OUTPUT);
  pinMode(PIN_PWM_L2, OUTPUT);
  pinMode(PIN_PWM_R1, OUTPUT);
  pinMode(PIN_PWM_R2, OUTPUT);
  pinMode(PIN_LED_LB, OUTPUT);
  pinMode(PIN_LED_RB, OUTPUT);

  delay(2000);
  stop_car();
  cmd.motion = Motion::Stop;
  cmd.v1 = 0;
  cmd.v2 = 0;
  cmd.dur = 0;
}

void loop()
{
  readsize = readline();
  if (readsize > 0){
    i = 0;
    cmd = parse_cmd(readbuf, readsize);
    startTime = millis();
    if (cmd.motion == Motion::Error) {
      debug_print_cmd(cmd);
      Serial.println("ERROR PARSING COMMAND!");
      return;
    }
    //debug_print_cmd(cmd);
    switch (cmd.motion) {
      case Motion::Right:
        turn_right(cmd.v1);
        break;
      case Motion::Forward:
        forward(cmd.v1);
        break;
      case Motion::Left:
        turn_left(cmd.v1);
        break;
      case Motion::Backward:
        reverse(cmd.v1);
        break;
      case Motion::Differential:
        differential(cmd.v1, cmd.v2);
        break;
      case Motion::Stop:
        stop_car();
        break;
     }
  }

  if (cmd.dur == 0){
    return;
  }
  
  currentTime = millis();
  timeDifference = currentTime - startTime;
  if (timeDifference >= cmd.dur){
    //delay(cmd.dur);
    stop_car();
    Serial.println("Complete");
    reset();
  }
  
}



/*
 * CAR SPECIFIC CODE HERE
 */
void stop_car()
{
  analogWrite(PIN_PWM_L1, 0);
  analogWrite(PIN_PWM_L2, 0);
  analogWrite(PIN_PWM_R1, 0);
  analogWrite(PIN_PWM_R2, 0);
}


void turn_right(int mag)
{
  analogWrite(PIN_PWM_L1, mag);
  analogWrite(PIN_PWM_L2, 0);
  analogWrite(PIN_PWM_R1, 0);
  analogWrite(PIN_PWM_R2, mag);
}

void turn_left(int mag)
{
  analogWrite(PIN_PWM_L1, 0);
  analogWrite(PIN_PWM_L2, mag);
  analogWrite(PIN_PWM_R1, mag);
  analogWrite(PIN_PWM_R2, 0);
}

void forward(int mag)
{
  analogWrite(PIN_PWM_L1, mag);
  analogWrite(PIN_PWM_L2, 0);
  analogWrite(PIN_PWM_R1, mag);
  analogWrite(PIN_PWM_R2, 0);
}

void reverse(int mag)
{
  analogWrite(PIN_PWM_L1, 0);
  analogWrite(PIN_PWM_L2, mag);
  analogWrite(PIN_PWM_R1, 0);
  analogWrite(PIN_PWM_R2, mag);
}

void differential(int v1, int v2)
{
  if (v1 >= 0) {
    analogWrite(PIN_PWM_L1, v1);
    analogWrite(PIN_PWM_L2, 0);
  } else {
    analogWrite(PIN_PWM_L1, 0);
    analogWrite(PIN_PWM_L2, v1);
  }

  if (v2 >= 0) {
    analogWrite(PIN_PWM_R1, v2);
    analogWrite(PIN_PWM_R2, 0);
  } else {
    analogWrite(PIN_PWM_R1, 0);
    analogWrite(PIN_PWM_R2, v2);
  }
}



/*
 * COMMAND PARSING SPECIFIC CODE HERE
 */
byte readline()
{
  size_t j;
  size_t readlen = Serial.available(); 
  for (j = 0; j < readlen; j++){
    readbuf[i] = Serial.read();
    if (readbuf[i] == '\n' || i == (READBUFLEN - 1)){
      readbuf[i] = '\0';
      Serial.print("Received the following line: ");
      Serial.println(readbuf);
      return i;
    }
    i++;
  }
  return 0;
}


Motion parse_motion(char mo)
{
  switch(mo) {
    case 'S': return Motion::Stop;
    case 'L': return Motion::Left;
    case 'R': return Motion::Right;
    case 'F': return Motion::Forward;
    case 'B': return Motion::Backward;
    case 'D': return Motion::Differential;
    default:  return Motion::Error;
  }
}


// Cmd String Format:
// M.SSS.D+
// ---- OR ----
// M.(+|-)SSS.(+|-)SSS.D+ (M = D for differential)
// M -> Motion (L, R, S, F, B)
// S -> Speed (0 <= SSS <= 255)
// D -> Duration (0 <= D+) { in millis }
Cmd parse_cmd(char *buf, size_t buflen)
{
  Cmd cmd;

  if (buflen < MINBUFLEN) {
    cmd.motion = Motion::Error;
    return cmd;
  }

  if ((cmd.motion = parse_motion(buf[0])) == Motion::Error)
    return cmd;

  if (cmd.motion != Motion::Differential) {
    // M.SSS '\0' D+
    buf[5] = '\0';
    cmd.v1 = atoi(&buf[2]);
    cmd.v2 = 0;
    cmd.dur = atoi(&buf[6]);
  } else {
    if (buflen < 13) {
      cmd.motion = Motion::Error;
      return cmd;
    }
    
    buf[6] = '\0';
    buf[11] = '\0';
    cmd.v1 = atoi(&buf[3]);
    cmd.v2 = atoi(&buf[8]);
    cmd.dur = atoi(&buf[12]);

    if (buf[2] == '-') cmd.v1 *= -1;
    if (buf[7] == '-') cmd.v2 *= -1;
  }

  return cmd;
}


void debug_print_cmd(Cmd &cmd)
{
  Serial.print("##################################\nDebug Print Cmd:\n\nMotion: ");
  switch(cmd.motion) {
    case Motion::Left:
      Serial.println("Left");
      break;
    case Motion::Right:
      Serial.println("Right");
      break;
    case Motion::Forward:
      Serial.println("Forward");
      break;
    case Motion::Backward:
      Serial.println("Backward");
      break;
    case Motion::Stop:
      Serial.println("Stop");
      break;
    case Motion::Differential:
      Serial.println("Differential");
      break;
    case Motion::Error:
      Serial.println("Error!\nEnding Print....\n####################################\n");
      return;
  }

  Serial.print("Speed (V1): ");
  Serial.println(cmd.v1);
  Serial.print("Speed (V2): ");
  Serial.println(cmd.v2);
  Serial.print("Duration (millis): ");
  Serial.println(cmd.dur);
  Serial.println("###################################");
}

void reset()
{
  cmd.motion = Motion::Stop;
  cmd.v1 = 0;
  cmd.v2 = 0;
  cmd.dur = 0;

  startTime = 0;
  currentTime = 0;
  timeDifference = 0;
}
