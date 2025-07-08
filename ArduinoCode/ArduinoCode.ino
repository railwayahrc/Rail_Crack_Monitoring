const int motorPWM = 9;
const int motorDir = 4;
const int light1Pin = 2;
const int light2Pin = 3;
const int armPin = 8; // New pin for arm control
int analogReadValue = 0;
int mappedValue = 0;
int counter = 0;
int speed = 0, dir = 0, l1 = 0, l2 = 0, arm = 0;

void setup() {
  Serial.begin(9600);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  pinMode(light1Pin, OUTPUT);
  pinMode(light2Pin, OUTPUT);
  pinMode(armPin, OUTPUT); // Set arm pin as output
  digitalWrite(motorDir, LOW);
  digitalWrite(light1Pin, LOW);
  digitalWrite(light2Pin, LOW);
  digitalWrite(armPin, LOW); // Arm is LOW by default
}

void loop() {
  analogReadValue = analogRead(A0);
  mappedValue = map(analogReadValue, 859, 757, 100, 0);
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Expect input format like: SPEED:120 DIR:1 L1:1 L2:0 ARM:1
    

    sscanf(input.c_str(), "SPEED:%d DIR:%d L1:%d L2:%d ARM:%d", &speed, &dir, &l1, &l2, &arm);

    if (arm) {
      digitalWrite(motorDir, dir ? HIGH : LOW);
      delay(500);
      digitalWrite(armPin, HIGH);
      analogWrite(motorPWM, constrain(speed, 0, 255));
    } else {
      speed = 0;
      digitalWrite(armPin, LOW);
      analogWrite(motorPWM, 0); // Always stop motor if not armed
    }

    // Lights can still be controlled independently
    digitalWrite(light1Pin, l1 ? HIGH : LOW);
    digitalWrite(light2Pin, l2 ? HIGH : LOW);

    delay(100); // Optional: debounce/avoid serial flooding if needed
    // Print current parameters
    Serial.print("Current parameters: ");
    Serial.print("SPEED:"); Serial.print(speed);
    Serial.print(" DIR:"); Serial.print(dir);
    Serial.print(" L1:"); Serial.print(l1);
    Serial.print(" L2:"); Serial.print(l2);
    Serial.print(" ARM:"); Serial.print(arm);
    Serial.print(" BATT:"); Serial.print(mappedValue); Serial.println(" %");
    delay(50);
    
  }
delay(200);
counter = counter + 1;
if (counter>= 50){
  counter = 0;
  Serial.print("Current parameters: ");
  Serial.print("SPEED:"); Serial.print(speed);
  Serial.print(" DIR:"); Serial.print(dir);
  Serial.print(" L1:"); Serial.print(l1);
  Serial.print(" L2:"); Serial.print(l2);
  Serial.print(" ARM:"); Serial.print(arm);
  Serial.print(" BATT:"); Serial.print(mappedValue); Serial.println(" %");

}


}

