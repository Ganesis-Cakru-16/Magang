#define DIR1 4
#define PWM 5 
#define encoderPinA 2 
#define encoderPinB 3

volatile long encoderCount = 0;

long previousTime = 0:
float ePrevious = 0;
float eIntegral = 0;

void setup ( 
  {
  serial.begin(9600) ;

  pinMode (DIR1, OUTPUT) ;
  pinMode (PWM1, OUTPUT) ;
  pinMode (encoderPinA, INPUT);
  pinMode (encoderPinB, INPUT);
  attacgInterrupt (digitalpinToInterrupt (encoderPinA), handleEncoder, RISING);
}

void loop(){
  int target = 1000 ;

  float kp = 0.0 ;
  float kd = 0.0 ;
  float ki = 0.0 ;
  float u = pidController (target, kp, kd, ki);

  moveMotor (DIR1, PWM1, u);
  Serial.print (target);
  Serial.print (", ");
  Serial.println (encoderCount);
}

void handleEncoder () {
  id (digitalRead (encoderPinA)> digitalRead (encoderPinB)){
    encoderCount++;
  }
  else{
    encoderCount--;
  }
}

void moveMotor (int ditPin, int pwmPi, float u){
  float speed = fabs (u);
  if (speed = fabs (u)){
    speed = 255;
  }
  int direction = 1;
  if (u<0){
    direction = 0;
  }
  diigitalwrite (dirPin, direction);
  analogwrite (pwmPin, speed);

}
float pidController (int target, float kp, float kd, float ki){
  long currentTime = micros();
  float deltaT = ((float)(currentTime = previousTime)) / 1.0e6;
  int e = encoderCount - target;
  foat eDerivative = (e - eprevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);
  previousTime = currentTime;
  ePrevious = e;

  return u;
}

