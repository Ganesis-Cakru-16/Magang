const int motorPin1 = 3;
const int motorPin2 = 4;
const int enablePin = 8;

// Define dumpSignal
const int dumpSignalPin = 2;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(dumpSignalPin, INPUT);
  
  // Default continuous brush
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(enablePin, 255);
}

void loop() {
  // Cek apabila dumpSignal aktif
  if (digitalRead(dumpSignalPin) == HIGH) {
    
    // Brush counterclockwise selama dumping
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 255);

    // Perkiraan waktu (disesuaikan dengan dumping)
    // Need testi apakah lebih baik brush akan tetap jalan counterclockwise setelah dumping
    
    // delay(5000);

    // After dumping, spin the motor forward again
    // digitalWrite(motorPin1, LOW); // Set motor direction
    // digitalWrite(motorPin2, HIGH); // Set motor direction
    // analogWrite(enablePin, 255); // Set motor speed (0-255)
  }

}
