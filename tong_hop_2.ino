// ==== Include Libraries ====
#include <SparkFun_TB6612.h>

// ==== Define Motor Pins ====
#define AIN1 5
#define BIN1 7
#define AIN2 6
#define BIN2 15
#define PWMA 4
#define PWMB 16
#define STBY1 17

#define AIN3 18
#define BIN3 46
#define AIN4 8
#define BIN4 9
#define PWMC 3
#define PWMD 10
#define STBY2 11

// ==== Cảm biến khoảng cách HC-SR04 ====
const int trig = 2;      // chân trig của HC-SR04
const int echo = 12;     // chân echo của HC-SR04

// ==== Setup dò line ====
const int NUM_SENSORS = 5;
const int sensor_pins[NUM_SENSORS] = {A4, A3, A2, A1, A0};

// ==== Điều kiển motor ====
const int offsetA = 1;
const int offsetB = 1;
const int BASE_SPEED = 100;  // Tốc độ cơ bản của robot

// ==== PID Parameters ====
float Kp = 25;           // Hệ số tỷ lệ
float Ki = 0.000;        // Hệ số tích phân
float Kd = 15;           // Hệ số vi phân

float error = 0;         // Sai số hiện tại
float lastError = 0;     // Sai số trước đó
float totalError = 0;    // Tổng sai số (cho thành phần I)
float deltaError = 0;    // Thay đổi sai số (cho thành phần D)

int leftSpeedAdjust = 0;  // Điều chỉnh tốc độ bánh trái
int rightSpeedAdjust = 0; // Điều chỉnh tốc độ bánh phải

unsigned long lastTime = 0;    // Thời điểm lần cuối tính PID
const int PID_INTERVAL = 50;   // Thời gian giữa các lần tính PID (ms)

// ==== Khởi tạo đối tượng motor ====
Motor motorFL(AIN1, AIN2, PWMA, offsetA, STBY1); // Front Left
Motor motorFR(BIN1, BIN2, PWMB, offsetB, STBY1); // Front Right
Motor motorBL(AIN3, AIN4, PWMC, offsetA, STBY2); // Back Left
Motor motorBR(BIN3, BIN4, PWMD, offsetB, STBY2); // Back Right

// ==== Setup Functions ====
void initMotors() {
  stopAll();
}

void initSensors() {
  // Khởi tạo cảm biến dò line
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], INPUT);
  }
  
  // Khởi tạo cảm biến khoảng cách HC-SR04
  pinMode(trig, OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo, INPUT);    // chân echo sẽ nhận tín hiệu
  
  Serial.begin(115200);
}

// ==== Hàm đọc cảm biến khoảng cách HC-SR04 ====
int readDistance() {
  unsigned long duration; // biến đo thời gian
  int distance;          // biến lưu khoảng cách
  
  /* Phát xung từ chân trig */
  digitalWrite(trig, LOW);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);  // phát xung từ chân trig
  delayMicroseconds(5);     // xung có độ dài 5 microSeconds
  digitalWrite(trig, LOW);   // tắt chân trig
  
  /* Tính toán thời gian */
  duration = pulseIn(echo, HIGH);  // Đo độ rộng xung HIGH ở chân echo
  distance = int(duration / 2 / 29.412);  // Tính khoảng cách đến vật (cm)
  
  Serial.print("Khoảng cách: ");
  Serial.print(distance);
  Serial.println("cm");
  
  return distance;
}

// ==== Hàm đọc cảm biến dò line ====
int readLineSensors() {
  int mask = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    mask = (mask << 1) | digitalRead(sensor_pins[i]);
  }

  int lineError = 100; // Mặc định là mất line

  // Xác định sai số dựa trên trạng thái cảm biến
  switch (mask) {
    case 0b11110: lineError = -4; break;  // Line nằm ngoài cùng bên trái
    case 0b11100: lineError = -3; break;  // Line nằm khá trái
    case 0b11101: lineError = -2; break;  // Line lệch trái
    case 0b11001: lineError = -1; break;  // Line hơi lệch trái
    case 0b11011: lineError = 0;  break;  // Line ở giữa
    case 0b10011: lineError = 1;  break;  // Line hơi lệch phải
    case 0b10111: lineError = 2;  break;  // Line lệch phải
    case 0b00111: lineError = 3;  break;  // Line nằm khá phải
    case 0b01111: lineError = 4;  break;  // Line nằm ngoài cùng bên phải
    case 0b11111: lineError = (lineError < 0) ? -5 : 5; break;  // Mất line
    default: break;
  }

  Serial.print("Mask: ");
  Serial.print(mask, BIN);
  Serial.print(" -> Error: ");
  Serial.println(lineError);

  return lineError;
}

// ==== Hàm tính toán PID ====
void calculatePID() {
  unsigned long currentTime = millis();
  
  // Chỉ tính PID sau mỗi khoảng thời gian PID_INTERVAL
  if (currentTime - lastTime >= PID_INTERVAL) {
    // Tính các thành phần PID
    float P = error;                          // Thành phần tỷ lệ
    totalError += error;                      // Cộng dồn sai số cho thành phần I
    deltaError = error - lastError;           // Tính độ thay đổi sai số cho thành phần D
    
    // Giới hạn thành phần I để tránh tích lũy quá lớn
    totalError = constrain(totalError, -50, 50);
    
    // Tính tổng điều khiển PID
    float pidValue = (Kp * P) + (Ki * totalError) + (Kd * deltaError);
    
    // Điều chỉnh tốc độ cho các động cơ
    leftSpeedAdjust = -pidValue;
    rightSpeedAdjust = pidValue;
    
    // Cập nhật các giá trị cho lần tính tiếp theo
    lastError = error;
    lastTime = currentTime;
    
    // Debug thông tin PID
    Serial.print("P: "); Serial.print(P);
    Serial.print(" I: "); Serial.print(totalError);
    Serial.print(" D: "); Serial.print(deltaError);
    Serial.print(" PID: "); Serial.println(pidValue);
  }
}

// ==== Các hàm điều khiển motor cơ bản ====
void stopAll() {
  brake(motorFL, motorFR);
  brake(motorBL, motorBR);
}

void moveForward(int baseSpeed) {
  // Áp dụng điều chỉnh PID vào tốc độ cơ bản
  int leftSpeed = constrain(baseSpeed + leftSpeedAdjust, 0, 255);
  int rightSpeed = constrain(baseSpeed + rightSpeedAdjust, 0, 255);
  
  forward(motorFL, leftSpeed);
  forward(motorFR, rightSpeed);
  forward(motorBL, leftSpeed);
  forward(motorBR, rightSpeed);
  
  // Debug tốc độ motor
  Serial.print("Left Speed: "); Serial.print(leftSpeed);
  Serial.print(" Right Speed: "); Serial.println(rightSpeed);
}

void moveBackward(int speed) {
  back(motorFL, speed);
  back(motorFR, speed);
  back(motorBL, speed);
  back(motorBR, speed);
}

void moveLeft(int speed) {
  forward(motorFL, speed);
  back(motorFR, speed);
  back(motorBL, speed);
  forward(motorBR, speed);
}

void moveRight(int speed) {
  back(motorFL, speed);
  forward(motorFR, speed);
  forward(motorBL, speed);
  back(motorBR, speed);
}

void rotateLeft(int speed) {
  back(motorFL, speed);
  forward(motorFR, speed);
  back(motorBL, speed);
  forward(motorBR, speed);
}

void rotateRight(int speed) {
  forward(motorFL, speed);
  back(motorFR, speed);
  forward(motorBL, speed);
  back(motorBR, speed);
}

void diagonalLeftForward(int speed) {
  forward(motorFL, speed);
  stop(motorFR);
  stop(motorBL);
  forward(motorBR, speed);
}

void diagonalRightForward(int speed) {
  stop(motorFL);
  forward(motorFR, speed);
  forward(motorBL, speed);
  stop(motorBR);
}

// ==== Hàm điều khiển robot ====
void controlRobot(int lineError) {
  // Cập nhật giá trị error cho PID
  error = lineError;
  
  // Tính toán giá trị điều khiển PID
  calculatePID();
  
  // Điều khiển robot dựa trên sai số
  switch (lineError) {
    case -5:
    case -4:
      moveLeft(BASE_SPEED); 
      break;

    case -3:
    case -2:
      diagonalLeftForward(BASE_SPEED); 
      break;

    case -1:
    case 0:
    case 1:
      // Sử dụng PID cho chuyển động thẳng
      moveForward(BASE_SPEED);
      break;

    case 2:
    case 3:
      diagonalRightForward(BASE_SPEED);
      break;

    case 4:
    case 5:
      moveRight(BASE_SPEED);
      break;

    default:
      stopAll();
      break;
  }
}

// ==== MAIN ====
void setup() {
  initMotors();
  initSensors();
  
  // Khởi tạo thời gian cho PID
  lastTime = millis();
}

void loop() {
  // Đọc khoảng cách từ cảm biến HC-SR04
  int distance = readDistance();
  
  // Đọc sai số từ cảm biến dò line
  int lineError = readLineSensors();
  
  // Điều khiển robot với PID
  controlRobot(lineError);
  
  // Delay nhỏ để ổn định hệ thống
  delay(10);
}
