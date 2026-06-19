// ---- Pin Definitions ----
#define TRIG_F 5
#define ECHO_F 18
#define TRIG_B 19
#define ECHO_B 21
#define TRIG_L 22
#define ECHO_L 23
#define TRIG_R 25
#define ECHO_R 26

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout (~5m max range)
  if (duration == 0) return -1; // no echo received
  long distance = duration * 0.0343 / 2; // convert to cm
  return distance;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
}

void loop() {
  long distF = readDistance(TRIG_F, ECHO_F);
  delay(50); // gap prevents cross-talk between sensors
  long distB = readDistance(TRIG_B, ECHO_B);
  delay(50);
  long distL = readDistance(TRIG_L, ECHO_L);
  delay(50);
  long distR = readDistance(TRIG_R, ECHO_R);
  delay(50);

  Serial.print("F: "); Serial.print(distF); Serial.print(" cm\t");
  Serial.print("B: "); Serial.print(distB); Serial.print(" cm\t");
  Serial.print("L: "); Serial.print(distL); Serial.print(" cm\t");
  Serial.print("R: "); Serial.print(distR); Serial.println(" cm");

  delay(100);
}
