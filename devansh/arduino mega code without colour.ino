/*
 * ═══════════════════════════════════════════════════════════════════════════
 * MAZE ROBOT — Arduino Mega 2560   (MECANUM STRAFE, BINARY PROTOCOL)
 * ═══════════════════════════════════════════════════════════════════════════
 */
#include <Arduino.h>

// ── Motor pins (L298N) ───────────────────────────────────
#define FL_EN 6
#define FL_IN1 30
#define FL_IN2 31
#define FR_EN 7
#define FR_IN1 32
#define FR_IN2 33
#define RR_EN 8
#define RR_IN1 34
#define RR_IN2 35
#define RL_EN 9
#define RL_IN1 36
#define RL_IN2 37

// ── Encoder pins (hardware interrupts: 2,3,18,19,20,21) ───
#define FL_ENC 2
#define RL_ENC 3
#define RR_ENC 21
#define FR_ENC 20

// ── Ultrasonic pins ──────────────────────────────────────
#define FRONT_TRIG 28
#define FRONT_ECHO 29
#define BACK_TRIG  24
#define BACK_ECHO  25
#define LEFT_TRIG  26
#define LEFT_ECHO  27
#define RIGHT_TRIG 22
#define RIGHT_ECHO 23

// ── Physical constants (TUNE THESE) ──────────────────────
#define TICKS_PER_REV   500
#define WHEEL_DIAM_MM   65
#define CELL_SIZE_MM    200
#define MM_PER_TICK     ((float)(3.14159f * WHEEL_DIAM_MM) / TICKS_PER_REV)
#define TICKS_PER_CELL  ((long)(CELL_SIZE_MM / MM_PER_TICK))
#define STRAFE_TICK_SCALE  1.0f

// ── PID ──
#define KP  2.5f
#define KI  0.05f
#define KD  0.8f

// ── Speeds (PWM 0-255) ───────────────────────────────────
#define DRIVE_SPEED  150
#define MIN_SPEED    60

// ── Sensor ───────────────────────────────────────────────
#define SENSOR_MAX_MM   450
#define SENSOR_TIMEOUT  25000
#define SENSOR_SAMPLES  3
#define UART_BAUD       115200

// ── Protocol ─────────────────────────────────────────────
#define PKT_H1 0xAA
#define PKT_H2 0x55
#define CMD_MOVE_FWD    0x01
#define CMD_TURN_LEFT   0x02
#define CMD_TURN_RIGHT  0x03
#define CMD_UTURN       0x04
#define CMD_STOP        0x05
#define CMD_GET_SENSORS 0x06
#define CMD_SET_SPEED   0x07

#define RSP_SENSOR_DATA 0x81
#define RSP_DONE        0x82
#define RSP_ERROR       0x84

#define STATUS_OK      0
#define STATUS_STALL   1
#define STATUS_TIMEOUT 2

#define MAX_PAYLOAD 16
struct Packet { uint8_t cmd; uint8_t len; uint8_t payload[MAX_PAYLOAD]; };

enum MotionStatus : uint8_t { MOT_OK = STATUS_OK, MOT_STALL = STATUS_STALL, MOT_TIMEOUT = STATUS_TIMEOUT };
struct WheelDir { int fl, fr, rl, rr; };

volatile long enc_fl = 0, enc_fr = 0, enc_rr = 0, enc_rl = 0;
volatile int8_t dir_fl = 1, dir_fr = 1, dir_rr = 1, dir_rl = 1;

void isr_fl() { enc_fl += dir_fl; }
void isr_rl() { enc_rl += dir_rl; }
void isr_rr() { enc_rr += dir_rr; }
void isr_fr() { enc_fr += dir_fr; }

void initEncoders() {
    pinMode(FL_ENC, INPUT_PULLUP); pinMode(RL_ENC, INPUT_PULLUP);
    pinMode(RR_ENC, INPUT_PULLUP); pinMode(FR_ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FL_ENC), isr_fl, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_ENC), isr_rl, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_ENC), isr_rr, RISING);
    attachInterrupt(digitalPinToInterrupt(FR_ENC), isr_fr, RISING);
}
void resetEncoders() { noInterrupts(); enc_fl = enc_fr = enc_rr = enc_rl = 0; interrupts(); }
static long absFL(){ noInterrupts(); long v=labs(enc_fl); interrupts(); return v; }
static long absFR(){ noInterrupts(); long v=labs(enc_fr); interrupts(); return v; }
static long absRR(){ noInterrupts(); long v=labs(enc_rr); interrupts(); return v; }
static long absRL(){ noInterrupts(); long v=labs(enc_rl); interrupts(); return v; }
static long absAvg(){ return (absFL()+absFR()+absRR()+absRL())/4; }

uint8_t g_driveSpeed = DRIVE_SPEED;

void setMotor(uint8_t en, uint8_t in1, uint8_t in2, int speed) {
    if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
    else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); speed = -speed; }
    else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
    analogWrite(en, (uint8_t)constrain(speed, 0, 255));
}
void setAllMotors(int fl, int fr, int rl, int rr) {
    dir_fl = (fl >= 0) ? 1 : -1;  dir_fr = (fr >= 0) ? 1 : -1;
    dir_rl = (rl >= 0) ? 1 : -1;  dir_rr = (rr >= 0) ? 1 : -1;
    setMotor(FL_EN, FL_IN1, FL_IN2, fl);
    setMotor(FR_EN, FR_IN1, FR_IN2, fr);
    setMotor(RL_EN, RL_IN1, RL_IN2, rl);
    setMotor(RR_EN, RR_IN1, RR_IN2, rr);
}
void stopAll() { setAllMotors(0,0,0,0); }
void initMotors() {
    uint8_t pins[] = { FL_EN,FL_IN1,FL_IN2, FR_EN,FR_IN1,FR_IN2,
                       RR_EN,RR_IN1,RR_IN2, RL_EN,RL_IN1,RL_IN2 };
    for (uint8_t p : pins) pinMode(p, OUTPUT);
    stopAll();
}

WheelDir dirFor(char move) {
    switch (move) {
        case 'F': return { +1, +1, +1, +1 };   // North
        case 'B': return { -1, -1, -1, -1 };   // South
        case 'R': return { +1, -1, -1, +1 };   // East
        case 'L': return { -1, +1, +1, -1 };   // West
    }
    return { 0,0,0,0 };
}

void initUltrasonics() {
    uint8_t trigs[] = {FRONT_TRIG, BACK_TRIG, LEFT_TRIG, RIGHT_TRIG};
    uint8_t echos[] = {FRONT_ECHO, BACK_ECHO, LEFT_ECHO, RIGHT_ECHO};
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(trigs[i], OUTPUT); pinMode(echos[i], INPUT);
        digitalWrite(trigs[i], LOW);
    }
}
static uint16_t rawPingMM(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long us = pulseIn(echo, HIGH, SENSOR_TIMEOUT);
    if (us == 0) return SENSOR_MAX_MM + 1;
    uint16_t mm = (uint16_t)(us * 0.17f);
    return (mm > SENSOR_MAX_MM) ? SENSOR_MAX_MM + 1 : mm;
}
uint16_t pingMM(uint8_t trig, uint8_t echo) {
    uint16_t s[SENSOR_SAMPLES];
    for (uint8_t i = 0; i < SENSOR_SAMPLES; i++){ s[i]=rawPingMM(trig,echo); delay(12); }
    for (uint8_t i = 1; i < SENSOR_SAMPLES; i++){ 
        uint16_t k=s[i]; int8_t j=i-1;
        while (j>=0 && s[j]>k){ s[j+1]=s[j]; j--; } s[j+1]=k;
    }
    return s[SENSOR_SAMPLES/2];
}

uint8_t calcCRC(uint8_t cmd, uint8_t len, const uint8_t* payload) {
    uint8_t crc = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) crc ^= payload[i];
    return crc;
}
void sendPacket(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    Serial.write(PKT_H1); Serial.write(PKT_H2);
    Serial.write(cmd);    Serial.write(len);
    for (uint8_t i = 0; i < len; i++) Serial.write(payload[i]);
    Serial.write(calcCRC(cmd, len, payload));
}
void sendSensors() {
    uint16_t f = pingMM(FRONT_TRIG, FRONT_ECHO);
    uint16_t b = pingMM(BACK_TRIG,  BACK_ECHO);
    uint16_t l = pingMM(LEFT_TRIG,  LEFT_ECHO);
    uint16_t r = pingMM(RIGHT_TRIG, RIGHT_ECHO);
    uint8_t p[8] = {
        (uint8_t)(f>>8),(uint8_t)f, (uint8_t)(b>>8),(uint8_t)b,
        (uint8_t)(l>>8),(uint8_t)l, (uint8_t)(r>>8),(uint8_t)r
    };
    sendPacket(RSP_SENSOR_DATA, p, 8);
}
void sendDone(uint8_t status) { sendPacket(RSP_DONE, &status, 1); }

static enum : uint8_t { S_H1,S_H2,S_CMD,S_LEN,S_PAYLOAD,S_CRC } rxState = S_H1;
static uint8_t rxCmd, rxLen, rxBuf[MAX_PAYLOAD], rxIdx;

bool receivePacket(Packet& out) {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        switch (rxState) {
            case S_H1:      if (b==PKT_H1) rxState=S_H2;                  break;
            case S_H2:      rxState = (b==PKT_H2)? S_CMD : S_H1;         break;
            case S_CMD:     rxCmd=b; rxState=S_LEN;                      break;
            case S_LEN:     rxLen=b; rxIdx=0;
                            rxState = (rxLen>0)? S_PAYLOAD : S_CRC;      break;
            case S_PAYLOAD: rxBuf[rxIdx++]=b;
                            if (rxIdx>=rxLen) rxState=S_CRC;              break;
            case S_CRC:
                rxState = S_H1;
                if (b == calcCRC(rxCmd, rxLen, rxBuf)) {
                    out.cmd=rxCmd; out.len=rxLen;
                    memcpy(out.payload, rxBuf, rxLen);
                    return true;
                }
                break;
        }
    }
    return false;
}

MotionStatus moveOneCell(char move) {
    WheelDir d = dirFor(move);
    if (d.fl==0 && d.fr==0 && d.rl==0 && d.rr==0) return MOT_OK;

    long target = (long)(TICKS_PER_CELL * ((move=='L'||move=='R') ? STRAFE_TICK_SCALE : 1.0f));
    resetEncoders();

    float errI = 0, errPrev = 0;
    uint32_t tStart = millis(), stallChk = millis();
    long lastAvg = 0;
    int base = g_driveSpeed;

    while (true) {
        long a = absAvg();
        if (a >= target) break;

        if (millis() - tStart > 5000UL) { stopAll(); return MOT_TIMEOUT; }
        if (millis() - stallChk > 300) {
            if (a == lastAvg) { stopAll(); return MOT_STALL; }
            lastAvg = a; stallChk = millis();
        }

        long diagA = absFL() + absRR();
        long diagB = absFR() + absRL();
        float err  = (float)(diagA - diagB);
        errI = constrain(errI + err, -200, 200);
        float errD = err - errPrev; errPrev = err;
        float corr = KP*err + KI*errI + KD*errD;

        int magA = constrain((int)(base - corr), MIN_SPEED, 255);
        int magB = constrain((int)(base + corr), MIN_SPEED, 255);
        setAllMotors(magA * d.fl, magB * d.fr, magB * d.rl, magA * d.rr);
        delay(5);
    }
    stopAll();
    delay(60);
    return MOT_OK;
}

MotionStatus moveCells(char move, uint8_t cells) {
    if (cells == 0) cells = 1;
    for (uint8_t i = 0; i < cells; i++) {
        MotionStatus st = moveOneCell(move);
        if (st != MOT_OK) return st;
    }
    return MOT_OK;
}

void setup() {
    Serial.begin(UART_BAUD);
    while (!Serial) {}
    initMotors();
    initEncoders();
    initUltrasonics();
}

void loop() {
    Packet pkt;
    if (!receivePacket(pkt)) return;

    MotionStatus result = MOT_OK;

    switch (pkt.cmd) {
        case CMD_GET_SENSORS:
            sendSensors();
            return;
        case CMD_MOVE_FWD: {
            uint8_t cells = (pkt.len > 0) ? pkt.payload[0] : 1;
            result = moveCells('F', cells);
            break;
        }
        case CMD_TURN_LEFT:  result = moveOneCell('L'); break;
        case CMD_TURN_RIGHT: result = moveOneCell('R'); break;
        case CMD_UTURN:      result = moveOneCell('B'); break;
        case CMD_STOP:       stopAll(); return;
        case CMD_SET_SPEED:
            if (pkt.len > 0) g_driveSpeed = pkt.payload[0];
            return;
        default:
            sendPacket(RSP_ERROR, (const uint8_t*)"\x01", 1);
            return;
    }

    sendSensors();
    sendDone((uint8_t)result);
}
