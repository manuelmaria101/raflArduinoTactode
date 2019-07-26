#include <TimerOne.h>
#include "channels.h"
#include "IRLine.h"
#include "proj_types.h"

/*---------------------------------------TACTODE STUFF----------------------------------*/
#define TIMEOUT 5000

typedef struct CmdMove {
  int t;
  int vel;
} cmdMove;
typedef struct CmdRotate {
  int angle;
  int vel;
} cmdRotate;

typedef struct Tile {
  char type;
  union Data {
    cmdMove Move;
    cmdRotate Rotate;
  } data;
} tile;


/*-----------------------------------------------------DECLARE VARIABLES---------------------------------------------*/

tile my_tile;
char program[] = "M 2000 50\nW 90 50\nM 2000 50\nW 90 50\nM 2000 50\nW 90 50\nM 2000 50\nW 90 50\nS";
int  i = 0;
float my_angle = 0;
byte next = 0;

/*-------------------------------------------------END OF TACTODE STUFF---------------------------*/

byte UsingSimulator;

channels_t serial_channels;
byte go;

IRLine_t IRLine;

// Encoder PINS
#define ENC1_A 7
#define ENC1_B 8

#define ENC2_A 11
#define ENC2_B 12

// Motor Pins
#define MOTOR1_IN1 6
#define MOTOR1_IN2 5
#define MOTOR1_PWM 9

#define MOTOR2_IN1 4
#define MOTOR2_IN2 3
#define MOTOR2_PWM 10

// Solenoid Pins
#define SOLENOID 2

// Touch Switch
#define TOUCHSW 13

void setSolenoidState(byte state)
{
  digitalWrite(SOLENOID, state);
}

byte readTouchSwitch(void)
{
  return digitalRead(TOUCHSW);
}

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

byte encoder1_state, encoder2_state;
int encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void EncodersInterrupt(void)
{
  byte next_state, table_input;

  // Encoder 1
  next_state  = digitalRead(ENC1_A) << 1;
  next_state |= digitalRead(ENC1_B);

  table_input = (encoder1_state << 2) | next_state;
  encoder1_pos += encoder_table[table_input];
  encoder1_state = next_state;

  // Encoder 2
  next_state  = digitalRead(ENC2_A) << 1;
  next_state |= digitalRead(ENC2_B);

  table_input = (encoder2_state << 2) | next_state;
  encoder2_pos -= encoder_table[table_input];
  encoder2_state = next_state;
}


void setMotorsVoltage(int v1, int v2)
{
  v1 = 4 * v1;
  if (v1 >= 0) {
    digitalWrite(MOTOR1_IN1, 1);
    digitalWrite(MOTOR1_IN2, 0);
    //analogWrite(MOTOR1_PWM, v1);
  } else {
    digitalWrite(MOTOR1_IN1, 0);
    digitalWrite(MOTOR1_IN2, 1);
    v1 = - v1;
    //analogWrite(MOTOR1_PWM, v1);
  }
  if (v1 > 1023) v1 = 1023;
  Timer1.pwm(MOTOR1_PWM, v1);

  v2 = 4 * v2;
  if (v2 >= 0) {
    digitalWrite(MOTOR2_IN1, 1);
    digitalWrite(MOTOR2_IN2, 0);
    //analogWrite(MOTOR2_PWM, v2);
  } else {
    digitalWrite(MOTOR2_IN1, 0);
    digitalWrite(MOTOR2_IN2, 1);
    v2 = - v2;
    //analogWrite(MOTOR2_PWM, v2);
  }
  if (v2 > 1023) v2 = 1023;
  Timer1.pwm(MOTOR2_PWM, v2);

}


void serial_write(uint8_t b)
{
  Serial.write(b);
}

robot_t robot;
byte TouchSwitch, LastTouchSwitch;

void setup()
{
  // Motors I/O Setup
  digitalWrite(MOTOR1_IN1, 0);
  digitalWrite(MOTOR1_IN2, 0);
  digitalWrite(MOTOR1_PWM, 0);
  digitalWrite(MOTOR2_IN1, 0);
  digitalWrite(MOTOR2_IN2, 0);
  digitalWrite(MOTOR2_PWM, 0);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  // Encoder I/O Setup
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  // Solenoid I/O Setup
  digitalWrite(SOLENOID, 0);
  pinMode(SOLENOID, OUTPUT);

  // Switch I/O Setup
  pinMode(TOUCHSW, INPUT_PULLUP);


  // Faster ADC - http://forum.arduino.cc/index.php/topic,6549.0.html
  // set prescaler to 16
  // sbi(ADCSRA,ADPS2); // cbi(ADCSRA,ADPS1); // cbi(ADCSRA,ADPS0);
  ADCSRA = (ADCSRA | (1 << ADPS2)) & ~((1 << ADPS1) | (1 << ADPS0));

  Timer1.initialize(200); // 5 KHz
  Timer1.attachInterrupt(EncodersInterrupt);

  Serial.begin(115200);
  serial_channels.init(process_serial_packet, serial_write);


  robot.b = 0.137 / 2;
  robot.r1 = 0.065 / 2;
  robot.r2 = 0.065 / 2;

  UsingSimulator = 0;

}


void readEncoders(void)
{
  cli();
  robot.enc1 = encoder1_pos;
  robot.enc2 = encoder2_pos;

  encoder1_pos = 0;
  encoder2_pos = 0;
  sei();

}


void readIRSensors(void)
{
  byte c;
  for (c = 0; c < 5; c++) {
    IRLine.IR_values[c] = 1023 - analogRead(A0 + 4 - c);
  }
}

uint32_t tis;
uint32_t current, previous, interval = 40000UL;


void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
  byte c;

  if (channel == 'o')  {  // Encoder reading
    robot.enc1 = value;
  } else if (channel == 'n')  {  // Encoder reading
    robot.enc2 = value;
  } else if (channel == 'k')  {  // Set wheelDistance
    robot.b = value / 1e6;

  } else if (channel == 'i')  {   // IR Sensors + Touch
    for (c = 0; c < 5; c++) {
      IRLine.IR_values[c] = 16 * ((value >> (c * 6)) & 0x3F);
    }
    TouchSwitch = ((value >> 31) & 1);

  } else if (channel == 'g')  {  // Control
    // Calc control
    go = 1;
  } else if (channel == 's')  {  // Set new state
    robot.state = value;
    if (robot.state == 1)  {
      robot.ds = 0;
      robot.dtheta = 0;
    }

  } else if (channel == 'p')  { // Ping
    obj.send(channel, value + 1);
    Serial.println(value + 1);
  }
}

void followLineRight(float Vnom, float K)
{
  robot.v = Vnom;
  robot.w = K * IRLine.pos_right;
}


void followLineLeft(float Vnom, float K)
{
  robot.v = Vnom;
  robot.w = K * IRLine.pos_left;
}

void followLine(float Vnom, float K)
{
  float pos;

  if (fabs(IRLine.pos_left) < fabs(IRLine.pos_right)) {
    pos = IRLine.pos_left;
  } else {
    pos = IRLine.pos_right;
  }

  robot.v = Vnom;
  robot.w = K * pos;
}


void moveRobot(float Vnom, float Wnom)
{
  robot.v = Vnom;
  robot.w = Wnom;
}


void setState(byte new_state)
{
  tis = 0;
  robot.state = new_state;
  robot.ds = 0;
  robot.dtheta = 0;
}


void odometry(void)
{
  float dt = 1e-6 * interval;

  robot.v1e = TWO_PI * robot.r1 / (2 * 1920.0  * dt) * robot.enc1;
  robot.v2e = TWO_PI * robot.r2 / (2 * 1920.0  * dt) * robot.enc2;

  robot.ve =  (robot.v1e + robot.v2e) / 2;
  robot.we =  (robot.v2e - robot.v1e) / (2 * robot.b);

  robot.ds += robot.ve * dt;
  robot.dtheta += robot.we * dt;
}

void control(void)
{
  if (next == 0)
  {
    readNewTile();
    next = 1;
  }
  else if (next != 0 && robot.state == 1 && tis >= my_tile.data.Move.t)
  {
    next = 0;
  }
  else if (next != 0 && robot.state == 2  && (robot.dtheta > my_angle))
  {
    next = 0;
  }



  if (robot.state == 1) //MOVE TILE
  {
    moveRobot(my_tile.data.Move.vel, 0);
  }
  else if (robot.state == 2) //ROTATE TILE
  {
    moveRobot(0, my_tile.data.Rotate.vel);
  }
  else if (robot.state == 0) //STOP TILE
  {
    moveRobot(0, 0);
  }




  /*------------------DEBUG----------------*/

  Serial.print(F("Robot State: "));
  Serial.print(robot.state);

  Serial.print(F("        tis: "));
  Serial.print(tis);

  Serial.print(F("        theta: "));
  Serial.print(robot.dtheta);

  Serial.print(F("        TYPE: "));
  Serial.print(my_tile.type);

  Serial.print(F("        t: "));
  Serial.print(my_tile.data.Move.t);
  Serial.print(F("        vel: "));
  Serial.print(my_tile.data.Move.vel);

  Serial.print(F("        angle: "));
  Serial.print(my_tile.data.Rotate.angle);
  Serial.print(F("        Rotate vel: "));
  Serial.print(my_tile.data.Rotate.vel);

  Serial.println();
}

void loop(void)
{
  if (UsingSimulator) {
    sim_loop();
  } else {
    real_loop();
  }
}


void sim_loop(void)
{
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    serial_channels.StateMachine(b);
    if (b == '!') UsingSimulator = 0;
    if (b == '#') UsingSimulator = 1;
  }

  if (go) {
    tis = tis + interval / 1000;
    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();
    odometry();
    control();
    go = 0;

    serial_channels.send('S',  robot.state);
    serial_channels.send('V',  round(robot.v * 1000));
    serial_channels.send('W',  round(robot.w * 1000));
    serial_channels.send('M',  robot.solenoid_state);
    serial_channels.send('X',  robot.dtheta / PI * 180);
    //serial_channels.send('Y',  IRLine.total);
    //serial_channels.send('Z',  IRLine.cross_count);



  }
}

void serial_print_format(int value, byte space)
{
  byte b, c;
  b = Serial.print(value);
  for (c = 0; c < space - b; c++) {
    Serial.print(" ");
  }
}


void real_loop(void)
{
  uint32_t t;
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    if (b == '+') robot.solenoid_state = 1; //setSolenoidState(1);
    if (b == '-') robot.solenoid_state = 0; //setSolenoidState(0);
    if (b == '(') {
      robot.v = 50;  //setMotorsVoltage(50, 50) ;
      robot.w =  0;
    }
    if (b == '/') {
      robot.v =  0;  //setMotorsVoltage(-200, 200) ;
      robot.w = 50;
    }
    if (b == '=') {
      robot.v =  0;  //setMotorsVoltage(200, -200) ;
      robot.w = -50;
    }
    if (b == ')') {
      robot.v = -50;  //setMotorsVoltage(-200, -200) ;
      robot.w = 0;
    }
    if (b == '?') {
      robot.v =  0;  //setMotorsVoltage(0, 0) ;
      robot.w = 0;
    }
    if (b == '\\') robot.state = 0;
    if (b == '*') robot.state = 1;
    if (b == '!') UsingSimulator = 0;
    if (b == '#') UsingSimulator = 1;
    serial_channels.StateMachine(b);
  }

  current = micros();
  if (current - previous >= interval) {
    previous = current;
    tis = tis + interval / 1000;

    readEncoders();

    t = micros();
    readIRSensors();
    t = micros() - t;

    LastTouchSwitch = TouchSwitch;
    TouchSwitch = readTouchSwitch();
    if (robot.state == 0 && LastTouchSwitch && !TouchSwitch) robot.state = 1;

    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();

    odometry();

    control();

    setSolenoidState(robot.solenoid_state);
    setMotorsVoltage(robot.v + robot.w, robot.v - robot.w);

    //return;
    byte c;
    /*
      Serial.print(F("Enc1: "));
      serial_print_format(robot.enc1, 4);

      Serial.print(F(" Enc2: "));
      serial_print_format(robot.enc2, 4);
    */
    /* for (c = 0; c < 5; c++) {
       Serial.print(" ");
       Serial.print(IRLine.IR_values[c]);
      }
      Serial.print(F(" PosR: "));
      Serial.print(IRLine.pos_right);

      Serial.print(F(" PosL: "));
      Serial.print(IRLine.pos_left);

      Serial.print(F(" Touch: "));
      Serial.print(TouchSwitch);

      Serial.print(F(" v: "));
      Serial.print(robot.v);

      Serial.print(F(" w: "));
      Serial.print(robot.w);

      Serial.print(F(" state: "));
      Serial.print(robot.state);

      Serial.print(F(" tis: "));
      Serial.print(tis);

      Serial.print(F(" ds: "));
      Serial.print(robot.ds);

      Serial.print(F(" dtheta: "));
      Serial.print(robot.dtheta / PI * 180);

      Serial.println();*/
  }

}

void readNewTile(void)
{
  char trash;
  char line[strlen(program)];
  int j;
  j = 0;
  for ( ; program[i] != '\0'; i++)
  {
    if (program[i] == '\n')
    {
      i++;
      break;
    }
    else
    {
      line[j] = program[i];
      j++;
    }
  }

  sscanf(line, "%c", &my_tile.type);
  Serial.println(line);
  if (my_tile.type == 'M')
  {
    sscanf(line, "%c %d %d", &trash, &my_tile.data.Move.t, &my_tile.data.Move.vel);
  }
  else if (my_tile.type == 'W')
  {
    sscanf(line, "%c %d %d", &trash, &my_tile.data.Rotate.angle, &my_tile.data.Rotate.vel);
    my_angle = refresh_angle(my_tile.data.Rotate.angle);
  }
  else if (my_tile.type == 'S')
  {
    my_tile.data.Move.t = 0;
    my_tile.data.Move.vel = 0;
    my_tile.data.Rotate.angle = 0;
    my_tile.data.Rotate.vel = 0;
  }
  chooseState();
}

void chooseState(void)
{
  if (my_tile.type == 'M')
  {
    setState(1);
  }
  else if (my_tile.type == 'W')
  {
    setState(2);
  }
  else if (my_tile.type == 'S')
  {
    setState(0);
  }
}

float refresh_angle(int my_angle)
{
  float new_angle = my_angle * PI / 180;
  return new_angle;
}
