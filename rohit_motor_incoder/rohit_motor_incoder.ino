//variables...............................
#define in1 6
#define in2 4
#define pwm 5
#define enb 8
#define A 2
#define B 3

int pos = 0;
//volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//int pwmVal  = 200;  //min = 122  and max = 255


// Setup...................................
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(A,INPUT);
pinMode(B,INPUT);
attachInterrupt(digitalPinToInterrupt(A),readEncoder, RISING);

}


//loop........................................
void loop() {
// set target position
// int target = 12000;
int target = 250*sin(prevT/1e6);

// PID constant

float kp = 1;
float kd = 0.025;
float ki = 0.0;

// time diffrence
long currT = micros();
float deltaT = ((float)(currT-prevT))/1.0e6;
prevT = currT; 

//// Read the position in an atomic block to avoid a potential
//  // misread if the interrupt coincides with this code running
//  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
//  int pos = 0; 
//  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//    pos = posi;
//  }

// error
int e = pos-target;

// derivative
float dedt = (e-eprev)/(deltaT);

// integral

float eintegral = eintegral + e *deltaT;

// control signal
float u = kp*e + kd*dedt + ki*eintegral;

// motor power

float pwr = fabs(u);
if(pwr>255){
  pwr = 255;
}

// motor direction

int dir = 1;
if (u<0){
  dir = -1;
}
// signal the motor
setMotor(dir,pwr,pwm,in1,in2);

// store previous error 
eprev = e;
Serial.print(target);
Serial.print(" ");
Serial.print(pos);
Serial.println();
}

//Functions ##############################################



void setMotor(int dir, int pwmVal, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH)
    digitalWrite(in2,LOW)
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


void readEncoder(){
  int pulseB = digitalRead(B);
  if(pulseB>0){
    pos++;
  }
  else{
    pos--;
  }
}

void motion(){
  setMotor(1,150,pwm,in1,in2);
  delay(1000);
  Serial.println(pos);
    
  setMotor(-1,150,pwm,in1,in2);
  delay(1000);
  Serial.println(pos);
  
  setMotor(0,150,pwm,in1,in2);
  delay(20);
  Serial.println(pos);
 
}
