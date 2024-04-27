//Define pin numbers for motors
#define enA1 5
#define enB1 6
#define in11 11
#define in21 10
#define in31 13
#define in41 12
int motorSpeedA1 = 0;
int motorSpeedB1 = 0;

//Define pin numbers for encoders
#define encoder1Pin 2
#define encoder2Pin 3

//Variables for encoder counts
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long previousEncoder=0;
volatile long currentEncoder;
long currentMillis=0;
long previousMillis=0;
//Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

void setup() {
  Serial.begin(9600);

  //Set pin modes
  pinMode(enA1, OUTPUT);
  pinMode(enB1, OUTPUT);
  pinMode(in11, OUTPUT);
  pinMode(in21, OUTPUT);
  pinMode(in31, OUTPUT);
  pinMode(in41, OUTPUT);
  pinMode(encoder1Pin, INPUT);
  pinMode(encoder2Pin, INPUT);
  
  //Interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoder1Pin), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2Pin), handleEncoder2, RISING);
}

void loop() {
  //Set desired setpoint for motor 2  
  int target = read_speed(encoder2Count);

  //Move motor 1
  motor2(true,150);

  //PID gains and computation
  float kp = 2.0;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);
  int pid = static_cast<int>(u);
  //Control motor 2 based on PID
  moveMotor(enA1, pid);

  //Print statements for debugging
  Serial.print(read_speed(encoder1Count));
  Serial.print(encoder1Count);
  Serial.print(", ");
  Serial.println(read_speed(encoder2Count));

}

//Functions called during interrupts
void handleEncoder1() {
  encoder1Count++;
}
void handleEncoder2() {
  encoder2Count++;
}

void moveMotor( int pwmPin, int u){
  //Maximum motor speed
  float speed = fabs(u);
  if (speed > 255){
    speed  = 255;
  }
  //Stop the motor during overshoot
  if (encoder2Count > encoder1Count){
    speed = 0;
  }
  //Control the motor
  int direction = 0;
  motor1(false,speed);
}

float pidController(int target, float kp, float kd, float ki) {
  //Measure time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  //Compute the error, derivative, and integral
  int e = target - read_speed(encoder1Count);
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;
  
  //Compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);
  
  //Update variables for the next iteration
  previousTime = currentTime;
  ePrevious = e;

  return u;
}
void motorcw1(int speed) {
    motorSpeedA1 = speed;
    analogWrite(enA1, motorSpeedA1);
    digitalWrite(in11, HIGH);
    digitalWrite(in21, LOW);
}
void motorccw1(int speed) {
    motorSpeedA1 = speed;
    analogWrite(enA1, motorSpeedA1);
    digitalWrite(in11, LOW);
    digitalWrite(in21, HIGH);
}
void motorcw2(int speed) {
    motorSpeedB1 = speed; // Perbarui kecepatan motor B1

    analogWrite(enB1, motorSpeedB1);
    digitalWrite(in31, HIGH);
    digitalWrite(in41, LOW);
}
void motorccw2(int speed) {
    motorSpeedB1 = speed; // Perbarui kecepatan motor B1
    analogWrite(enB1, motorSpeedB1);
    digitalWrite(in31, LOW);
    digitalWrite(in41, HIGH);
}
void motor1(bool dir,int pwm){
  if (dir) {
    motorcw1(pwm);
  }
  else {
    motorccw1(pwm);
  }
}
void motor2(bool dir,int pwm){
  if (dir) {
    motorcw2(pwm);
  }
  else {
    motorccw2(pwm);
  }
}
float read_speed(int encoder)
{
    //read velocity of selected motor
    //return velocity in rad/s
    const int Encoder_1_round = 44; //define number of pulses in one round of encoder
    int currentEncoder;
    currentEncoder = encoder;
    
    float rot_speed;           //rotating speed in rad/s
    const int interval = 1000; //choose interval is 1 second (1000 milliseconds)

    currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
        previousMillis = currentMillis;
        rot_speed = (float)((currentEncoder - previousEncoder)*60 / (Encoder_1_round));
        previousEncoder = currentEncoder;
        return rot_speed;
    }
}