const int motorMajuPin = 5;
const int motorMundurPin = 6;
const int enableMotorPin = 9;

// Sensor bin penuh (disesuaikan dengan comvis)
const int fullBinSensorPin = 7;

// Define untuk bagian mulut
const int dumpSignalPin = fullBinSensorPin;

const int motorSpeed = 255;
const int dumpDuration = 5000;
const int retractDuration = 5000;
const int cleaningDuration = 7000;

void setup() {
  pinMode(motorMajuPin, OUTPUT);
  pinMode(motorMundurPin, OUTPUT);
  pinMode(enableMotorPin, OUTPUT);

  pinMode(fullBinSensorPin, INPUT);
}

void loop() {
  // Cek bin penuh
  if (digitalRead(fullBinSensorPin) == HIGH) {
    // Motor maju (proses dumping)
    digitalWrite(motorMajuPin, HIGH);
    digitalWrite(motorMundurPin, LOW);
    analogWrite(enableMotorPin, motorSpeed);
    
    // Perkiraan waktu dumping
    delay(dumpDuration);
    
    // Stop
    digitalWrite(motorMajuPin, LOW);
    digitalWrite(motorMundurPin, LOW);
    analogWrite(enableMotorPin, 0);

    delay(cleaningDuration); // Perkiraan mulut selesai membuang sampah
    
    // Motor mundur (proses retract)
    digitalWrite(motorMajuPin, LOW);
    digitalWrite(motorMundurPin, HIGH);
    analogWrite(enableMotorPin, motorSpeed);
    
    // Perkiraan waktu retract
    delay(retractDuration);
    
    // Kembali ke posisi semula
    digitalWrite(motorMajuPin, LOW);
    digitalWrite(motorMundurPin, LOW);
    analogWrite(enableMotorPin, 0);
  }
  
}
