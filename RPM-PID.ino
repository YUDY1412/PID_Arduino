#include <util/atomic.h>
#include <TimerOne.h>

//****PID ANGULAR SPEED CONTROL FOR BRUSHED DC MOTOR UNDER NO LOAD*****
//***********ONE DIRECTION-CLOSED LOOP-USING ARDUINO UNO***************
//***********ENTER YOUR PID PARAMETERS BELOW***************************
float kp =  2.4766;  //change value of Kp
float ki = 3.5677; //change value of Ki
float kd = 0.29438; // change value of Kd

// Error signals and PID (proportional Integral Derivative) terms
float error_now, error_prev = 0, integ_now, integ_prev = 0;

#define ENCA 18
#define ENCB 19
#define IN1 7
#define IN2 8
#define PWM 6

long t_prev = 0;
int pos_prev = 0;

volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long t_i = 0;
float v1Prev = 0;
float v1Filt = 0;

int c_i = 0;
int pwr = 0;
int dir = 0;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  pos_i = pos_i + increment;

}

void Change_RPM()
{
  c_i++;
}

void setup()
{
  Serial.begin(9600);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Timer1.initialize(3000000);
  Timer1.attachInterrupt(Change_RPM);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
 // delay(1000);
}

void loop()
{
  float RMP_in = 45;
  int pos = 0;
  int c = 0;
  long t_now = micros();

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    pos = pos_i;
    c = c_i;
  }

  // pwr = 250;
  // dir = 1;
  // RMP_in = (float)(pwr)*50/255;

  float t_delta = ((float)(t_now - t_prev)) / 1.0e6;
  float velocity1 = (pos - pos_prev) / t_delta;
  float v1 = (velocity1 / 1001) * 60;

  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;

  error_now = RMP_in - v1Filt;
  integ_now = integ_prev + (t_delta * (error_now + error_prev) / 2);
  //doan nay ban phai doi theo dong co(he so giam toc)
  //dong co toi 50rpm he so giam toc la 78, 11 xung encoder 1 vong nen chia 78*11=858

  pwr = kp * error_now + ki * integ_now + (kd * (error_now - error_prev) / t_delta) ;
  if (pwr >= 0) dir = 1;
  else dir = -1;

  pwr = abs(pwr);

  setMotor(dir, pwr, PWM, IN1, IN2);
  //Serial.print(dir * pwr);
  //Serial.print(",");
  Serial.print(v1Filt);
  Serial.println();

  integ_prev = integ_now;
  error_prev = error_now;
  pos_prev = pos;
  t_prev = t_now;

}
