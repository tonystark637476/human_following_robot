// ═══════════════════════════════════════════════════════════════
//  HUMAN FOLLOWING ROBOT — Arduino Nano
//  All 3 sensors on front: M=straight, L=45°left, R=45°right
//  Steering: Dl<Dm → left | Dr<Dm → right (background-proof)
//  Leg gap fix: spike rejection + rolling average + confirm count
// ═══════════════════════════════════════════════════════════════

// ── Motor Driver Pins (L298N) ──────────────────────────────────
const int IN1 = 2;    // left  motor forward
const int IN2 = 3;    // left  motor backward
const int IN3 = 4;    // right motor forward
const int IN4 = 5;    // right motor backward
const int ENA = 6;    // left  motor speed  — PWM ✔
const int ENB = 9;    // right motor speed  — PWM ✔ (NOT D7 on Nano!)

// ── Ultrasonic Sensor Pins ─────────────────────────────────────
const int TRIGM = 8;  const int ECHOM = 10;  // M — straight forward
const int TRIGL = 11; const int ECHOL = 12;  // L — 45° left
const int TRIGR = 13; const int ECHOR = 7;   // R — 45° right

// ── Distance Thresholds (cm) ───────────────────────────────────
const float TOO_CLOSE   = 15.0;  // back up if closer than this
const float FOLLOW_DIST = 30.0;  // ideal follow distance
const float TOO_FAR     = 50.0;  // chase if farther than this
const float MAX_DIST    = 400.0;
const float MIN_DIST    = 2.0;

// ── Speed (0–255) ──────────────────────────────────────────────
const int SPEED_FULL = 200;
const int SPEED_SLOW = 130;
const int SPEED_TURN = 160;

// ── Steering ───────────────────────────────────────────────────
// L or R must read this much closer than M to trigger a turn.
// Raise if robot turns too easily, lower if slow to react.
const float STEER_MARGIN = 8.0;

// ── Leg Gap Protection ─────────────────────────────────────────
const float SPIKE_THRESHOLD = 50.0; // max believable jump per cycle
const int   CONFIRM_COUNT   = 3;    // consecutive far reads to confirm
const int   SMOOTH_SIZE     = 6;    // rolling average window size

// ── State Variables ────────────────────────────────────────────
float Dm, Dl, Dr;
float lastDm = 30.0, lastDl = 42.0, lastDr = 42.0;
float histM[SMOOTH_SIZE], histL[SMOOTH_SIZE], histR[SMOOTH_SIZE];
int   hIdx     = 0;
int   farCount = 0;

// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Sensor pins
  pinMode(TRIGM, OUTPUT); pinMode(ECHOM, INPUT);
  pinMode(TRIGL, OUTPUT); pinMode(ECHOL, INPUT);
  pinMode(TRIGR, OUTPUT); pinMode(ECHOR, INPUT);

  // Fill history buffers with safe defaults
  // L and R default slightly higher than M because
  // 45° angled sensors see longer diagonal distance
  for (int i = 0; i < SMOOTH_SIZE; i++) {
    histM[i] = 30.0;
    histL[i] = 42.0;  // 30 / cos(45°) ≈ 42 cm
    histR[i] = 42.0;
  }

  stopMotors();
  delay(500);
  Serial.println("Robot ready!");
}

// ══════════════════════════════════════════════════════════════
void loop() {
  // Read all 3 sensors with filtering
  Dm = filteredRead(TRIGM, ECHOM, histM, lastDm);
  Dl = filteredRead(TRIGL, ECHOL, histL, lastDl);
  Dr = filteredRead(TRIGR, ECHOR, histR, lastDr);

  // Save for next spike check
  lastDm = Dm;
  lastDl = Dl;
  lastDr = Dr;

  // Debug output
  Serial.print("Dm:"); Serial.print(Dm, 1);
  Serial.print(" Dl:"); Serial.print(Dl, 1);
  Serial.print(" Dr:"); Serial.println(Dr, 1);

  navigate();
  delay(60);  // HC-SR04 needs 60ms between cycles
}

// ══════════════════════════════════════════════════════════════
//  NAVIGATE — main decision tree
// ══════════════════════════════════════════════════════════════
void navigate() {
  hIdx = (hIdx + 1) % SMOOTH_SIZE;

  // ── CASE 1: Too close — reverse immediately ────────────────
  if (Dm > MIN_DIST && Dm < TOO_CLOSE) {
    farCount = 0;
    backward(SPEED_FULL);
    Serial.println(">> BACKWARD");
    return;
  }

  // ── CASE 2: Sweet spot — hold position ────────────────────
  if (Dm >= TOO_CLOSE && Dm <= FOLLOW_DIST) {
    farCount = 0;
    stopMotors();
    Serial.println(">> HOLD");
    return;
  }

  // ── CASE 3: Approaching zone — slow forward + steer ───────
  if (Dm > FOLLOW_DIST && Dm <= TOO_FAR) {
    farCount = 0;
    steer(SPEED_SLOW);
    return;
  }

  // ── CASE 4: Far — wait for confirmation (leg gap fix) ─────
  if (Dm > TOO_FAR) {
    farCount++;
    Serial.print(">> FAR (");
    Serial.print(farCount);
    Serial.print("/");
    Serial.print(CONFIRM_COUNT);
    Serial.println(")");

    if (farCount >= CONFIRM_COUNT) {
      steer(SPEED_FULL);   // confirmed — person walked away
    } else {
      stopMotors();        // waiting — could be leg gap
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  STEER — Dl<Dm turns left | Dr<Dm turns right
//  Comparing against Dm cancels out background objects
// ══════════════════════════════════════════════════════════════
void steer(int spd) {
  bool personLeft  = (Dl < Dm - STEER_MARGIN);
  bool personRight = (Dr < Dm - STEER_MARGIN);

  if (personLeft && !personRight) {
    turnLeft(spd);
    Serial.println(">> TURN LEFT  (Dl < Dm)");
  }
  else if (personRight && !personLeft) {
    turnRight(spd);
    Serial.println(">> TURN RIGHT (Dr < Dm)");
  }
  else {
    // Both or neither — go straight
    forward(spd);
    Serial.println(">> STRAIGHT");
  }
}

// ══════════════════════════════════════════════════════════════
//  FILTERED READ
//  Layer 1 — spike rejection  (leg gap single frame fix)
//  Layer 2 — rolling average  (smooths remaining noise)
// ══════════════════════════════════════════════════════════════
float filteredRead(int trig, int echo, float* hist, float lastGood) {
  float raw = getDistance(trig, echo);

  // Layer 1: reject impossible jumps
  if (raw < 0 || abs(raw - lastGood) > SPIKE_THRESHOLD) {
    raw = lastGood;
  }

  // Layer 2: push into rolling buffer
  hist[hIdx] = raw;
  float sum = 0;
  for (int i = 0; i < SMOOTH_SIZE; i++) sum += hist[i];
  return sum / SMOOTH_SIZE;
}

// ══════════════════════════════════════════════════════════════
//  DISTANCE — median of 3 raw readings
// ══════════════════════════════════════════════════════════════
float getDistance(int trig, int echo) {
  float s[3];
  int valid = 0;

  for (int i = 0; i < 3; i++) {
    float d = singleRead(trig, echo);
    if (d >= MIN_DIST && d <= MAX_DIST) s[valid++] = d;
    delay(8);
  }

  if (valid == 0) return -1;
  if (valid == 1) return s[0];
  if (valid == 2) return (s[0] + s[1]) / 2.0;

  // Sort and return median
  if (s[0] > s[1]) { float t = s[0]; s[0] = s[1]; s[1] = t; }
  if (s[1] > s[2]) { float t = s[1]; s[1] = s[2]; s[2] = t; }
  if (s[0] > s[1]) { float t = s[0]; s[0] = s[1]; s[1] = t; }
  return s[1];
}

float singleRead(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 25000);
  if (dur == 0) return -1;
  return (0.034 * dur) / 2.0;
}

// ══════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ══════════════════════════════════════════════════════════════
void forward(int spd) {
  analogWrite(ENA, spd); analogWrite(ENB, spd);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void backward(int spd) {
  analogWrite(ENA, spd); analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void turnLeft(int spd) {
  analogWrite(ENA, spd); analogWrite(ENB, spd);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // left  motor reverse
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // right motor forward
}

void turnRight(int spd) {
  analogWrite(ENA, spd); analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // left  motor forward
  digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW); // right motor reverse
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}