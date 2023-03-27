enum class Motion { Stop, Forward, Backward, Left, Right, Error };

typedef struct {
  Motion motion;
  byte spd;
  size_t dur;
} Cmd;

#define READBUFLEN 20
#define MINBUFLEN 7
char readbuf[READBUFLEN];

byte readline()
{
  byte i;
  size_t readlen;

  i = 0;
  while(true) {
    if (Serial.available()) {
      readbuf[i] = Serial.read();
      if (readbuf[i++] == '\n') break;
      if (i == (READBUFLEN - 1)) break;
    } else {
      // This part may not be necessary, not sure though
      delay(1);
    }
  }

  readbuf[i] = '\0';
  Serial.print("Received the following line: ");
  Serial.println(readbuf);
  return i;
}


Motion parse_motion(char mo)
{
  switch(mo) {
    case 'S': return Motion::Stop;
    case 'L': return Motion::Left;
    case 'R': return Motion::Right;
    case 'F': return Motion::Forward;
    case 'B': return Motion::Backward;
    default:  return Motion::Error;
  }
}


// Cmd String Format:
// M.SSS.D+
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

  buf[5] = '\0';
  cmd.spd = atoi(&buf[2]);
  cmd.dur = atoi(&buf[6]);

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
    case Motion::Error:
      Serial.println("Error!\nEnding Print....\n####################################\n");
      return;
  }

  Serial.print("Speed: ");
  Serial.println(cmd.spd);
  Serial.print("Duration (millis): ");
  Serial.println(cmd.dur);
  Serial.println("###################################");
}


void setup() {
  Serial.begin(9600);
}


void loop() {
  Cmd cmd;
  byte readsize;

  readsize = readline();
  cmd = parse_cmd(readbuf, readsize);
  debug_print_cmd(cmd);
}
