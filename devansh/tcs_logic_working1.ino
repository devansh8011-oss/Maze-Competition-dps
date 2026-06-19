#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int R = 0;
int G = 0;
int B = 0;

void setup() {

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // 20% frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
}

void loop() {

  // ===== RED =====
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  R = pulseIn(sensorOut, LOW);

  // ===== GREEN =====
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  G = pulseIn(sensorOut, LOW);

  // ===== BLUE =====
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  B = pulseIn(sensorOut, LOW);

  // ===== PRINT RAW VALUES =====
  Serial.print("R:");
  Serial.print(R);

  Serial.print(" G:");
  Serial.print(G);

  Serial.print(" B:");
  Serial.print(B);

  Serial.print("  ->  ");

  // ==================================================
  // IMPORTANT:
  // LOWER VALUE = STRONGER COLOR
  // ==================================================

  // ===== BLUE =====
  if (B < R && B < G && B < 80) {
    Serial.println("BLUE");
  }

  // ===== BLACK =====
  else if (R > 150 && G > 150 && B > 150) {
    Serial.println("BLACK");
  }

  // ===== WHITE =====
  else if (R < 45 && G < 45 && B < 45) {
    Serial.println("WHITE");
  }

  // ===== SILVER =====
  // ===== SILVER =====
else if (

    R > 35 && R < 120 &&
    G > 35 && G < 120 &&
    B > 35 && B < 120 &&

    abs(R - G) < 25 &&
    abs(G - B) < 25 &&
    abs(R - B) < 25 &&

    !(R < 45 && G < 45 && B < 45)

) {

  Serial.println("SILVER");
}

  // ===== NONE =====
  else {
    Serial.println("NO REQUIRED COLOUR");
  }

  delay(500);
}
