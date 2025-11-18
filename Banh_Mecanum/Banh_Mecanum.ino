/*
======================================================================
  ARDUINO MEGA 2560 + 2x L298N + BLUETOOTH HC-06
  ROBOT BÁNH MECANUM ĐIỀU KHIỂN QUA UART3
  Tác giả: TranDangKhoaTechnology
======================================================================

📌 MỤC ĐÍCH:
  - Điều khiển xe 4 bánh Mecanum bằng 2 driver L298N.
  - Nhận lệnh điều khiển từ điện thoại/PC qua Bluetooth HC-06.
  - Giao tiếp Arduino Mega với HC-06 bằng UART3 (Serial3).

📌 TÍNH NĂNG CHÍNH:
  - F / B / L / R  : Tiến / lùi / quay trái / quay phải (xoay tại chỗ).
  - G / I / H / J  : Rẽ mềm (đi xéo, chỉnh tốc bánh trong/ngoài cua bằng diagFactor).
  - Q / E          : Đi ngang (strafe) sang trái / phải dùng cơ chế Mecanum.
  - + / -          : Tăng / giảm tốc độ từng bước (SPEED_STEP).
  - 0..9           : Chọn nhanh mức tốc độ (0 = chậm nhất, 9 = nhanh nhất).
  - S              : Dừng tự do (thả động cơ, không phanh).
  - P              : Phanh cứng (brake điện, INx = HIGH, PWM = 0).

📌 THÔNG SỐ TỐC ĐỘ:
  - PWM từ 0 đến 255.
  - currentSpeed : tốc độ hiện tại áp dụng cho tất cả lệnh di chuyển.
  - SPEED_MIN    : tốc độ nhỏ nhất cho phép.
  - SPEED_MAX    : tốc độ lớn nhất cho phép.
  - diagFactor   : hệ số giảm tốc bánh trong cua (0.0 ~ 1.0) dùng cho G/I/H/J.

📌 PHẦN CỨNG:
  - 01 x Arduino Mega 2560.
  - 02 x L298N (mỗi con điều khiển 2 motor DC → tổng 4 motor).
  - 01 x Bluetooth HC-06 (UART; TX → RX3, RX → TX3).
  - 04 x Động cơ DC gắn bánh Mecanum (FL, FR, RL, RR).
  - Nguồn động cơ 6–12 V (GND chung với Arduino).

======================================================================
  SƠ ĐỒ NỐI DÂY (TỔNG QUAN)
======================================================================

  L298N #1 (TRƯỚC – FRONT)
    - Motor 1 (M1 – Front Left  – FL) : ENA1, IN11, IN21
    - Motor 2 (M2 – Front Right – FR) : ENB1, IN31, IN41

  L298N #2 (SAU – REAR)
    - Motor 3 (M3 – Rear Left   – RL) : ENA2, IN12, IN22
    - Motor 4 (M4 – Rear Right  – RR) : ENB2, IN32, IN42

  Mapping chân Arduino:

    ENA1 =  5        ENB1 =  4
    ENA2 =  3        ENB2 =  2

    IN11 = 36        IN21 = 34      (M1 - Front Left - FL)
    IN31 = 32        IN41 = 30      (M2 - Front Right - FR)

    IN12 = 28        IN22 = 26      (M3 - Rear Left - RL)
    IN32 = 24        IN42 = 22      (M4 - Rear Right - RR)

📌 LƯU Ý NGUỒN:
  - Nguồn động cơ (12V…) cấp vào chân +12V của L298N.
  - GND nguồn động cơ phải nối chung GND với Arduino.
  - KHÔNG dùng 5V từ Arduino để cấp trực tiếp cho motor.

======================================================================
  GIAO TIẾP BLUETOOTH:
======================================================================
  - HC-06 TX → Arduino RX3 (chân 15).
  - HC-06 RX → Arduino TX3 (chân 14) (nên dùng mạch chia áp nếu cần).
  - Baud mặc định: 9600 bps (Serial3.begin(9600)).

======================================================================
  BẮT ĐẦU PHẦN CODE
======================================================================
*/

#include <Arduino.h>

// ================== KHAI BÁO CHÂN L298N (2 BOARD) ==================
// L298N số 1 - Điều khiển 2 bánh TRƯỚC (Front)
#define ENA1 5   // PWM cho motor M1 (Front Left)
#define ENB1 4   // PWM cho motor M2 (Front Right)

#define IN11 36  // IN1  - điều khiển chiều M1
#define IN21 34  // IN2  - điều khiển chiều M1
#define IN31 32  // IN3  - điều khiển chiều M2
#define IN41 30  // IN4  - điều khiển chiều M2

// L298N số 2 - Điều khiển 2 bánh SAU (Rear)
#define ENA2 3   // PWM cho motor M3 (Rear Left)
#define ENB2 2   // PWM cho motor M4 (Rear Right)

#define IN12 28  // IN1  - điều khiển chiều M3
#define IN22 26  // IN2  - điều khiển chiều M3
#define IN32 24  // IN3  - điều khiển chiều M4
#define IN42 22  // IN4  - điều khiển chiều M4

// ================== THAM SỐ TỐC ĐỘ ==================
int currentSpeed = 150;          // Tốc độ mặc định ban đầu (0–255)
const int SPEED_STEP = 20;       // Bước tăng/giảm tốc khi bấm '+' hoặc '-'
const int SPEED_MIN  = 60;       // Tốc độ thấp nhất cho phép (để motor vẫn quay được)
const int SPEED_MAX  = 255;      // Tốc độ cao nhất cho phép

// Hệ số giảm tốc bánh trong cua cho các lệnh G/I/H/J
//  - 0.3 → cua rất gắt, bánh trong cua rất chậm
//  - 0.5 → trung bình (50% tốc độ bánh ngoài)
//  - 0.7 → cua nhẹ, ít chênh lệch giữa hai bên
float diagFactor = 0.5;          

// ================== KHỞI TẠO CHÂN MOTOR ==================
void initMotorPins() {
  // Chân PWM (ENA, ENB) của 2 L298N
  pinMode(ENA1, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ENB2, OUTPUT);

  // Chân điều khiển chiều quay INx
  pinMode(IN11, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(IN31, OUTPUT);
  pinMode(IN41, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(IN32, OUTPUT);
  pinMode(IN42, OUTPUT);

  // Tắt toàn bộ output ban đầu (xe đứng yên)
  digitalWrite(IN11, LOW);
  digitalWrite(IN21, LOW);
  digitalWrite(IN31, LOW);
  digitalWrite(IN41, LOW);
  digitalWrite(IN12, LOW);
  digitalWrite(IN22, LOW);
  digitalWrite(IN32, LOW);
  digitalWrite(IN42, LOW);

  // PWM = 0 → không cấp điện cho motor
  analogWrite(ENA1, 0);
  analogWrite(ENB1, 0);
  analogWrite(ENA2, 0);
  analogWrite(ENB2, 0);
}

// ================== ĐIỀU KHIỂN 1 MOTOR ==================
// motorId: 0=M1(FL), 1=M2(FR), 2=M3(RL), 3=M4(RR)
// speed  : -255..255 (dương = tiến, âm = lùi, 0 = dừng)
void setOneMotor(uint8_t motorId, int speed) {
  // Giới hạn speed trong khoảng -255..255
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  uint8_t pwm = abs(speed);   // duty PWM (0..255)

  switch (motorId) {
    // ---------------- M1 - Front Left ----------------
    case 0: {
      if (speed > 0) {
        // Quay thuận
        digitalWrite(IN11, HIGH);
        digitalWrite(IN21, LOW);
      } else if (speed < 0) {
        // Quay nghịch
        digitalWrite(IN11, LOW);
        digitalWrite(IN21, HIGH);
      } else {
        // Dừng (free-run)
        digitalWrite(IN11, LOW);
        digitalWrite(IN21, LOW);
      }
      analogWrite(ENA1, pwm); // PWM cho M1
      break;
    }

    // ---------------- M2 - Front Right ----------------
    case 1: {
      if (speed > 0) {
        digitalWrite(IN31, HIGH);
        digitalWrite(IN41, LOW);
      } else if (speed < 0) {
        digitalWrite(IN31, LOW);
        digitalWrite(IN41, HIGH);
      } else {
        digitalWrite(IN31, LOW);
        digitalWrite(IN41, LOW);
      }
      analogWrite(ENB1, pwm); // PWM cho M2
      break;
    }

    // ---------------- M3 - Rear Left ----------------
    case 2: {
      if (speed > 0) {
        digitalWrite(IN12, HIGH);
        digitalWrite(IN22, LOW);
      } else if (speed < 0) {
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, HIGH);
      } else {
        digitalWrite(IN12, LOW);
        digitalWrite(IN22, LOW);
      }
      analogWrite(ENA2, pwm); // PWM cho M3
      break;
    }

    // ---------------- M4 - Rear Right ----------------
    case 3: {
      if (speed > 0) {
        digitalWrite(IN32, HIGH);
        digitalWrite(IN42, LOW);
      } else if (speed < 0) {
        digitalWrite(IN32, LOW);
        digitalWrite(IN42, HIGH);
      } else {
        digitalWrite(IN32, LOW);
        digitalWrite(IN42, LOW);
      }
      analogWrite(ENB2, pwm); // PWM cho M4
      break;
    }

    default:
      // Không có motorId hợp lệ
      break;
  }
}

// ================== ĐIỀU KHIỂN CẢ 4 MOTOR ==================
// m1..m4 : -255..255 cho từng bánh
void setAllMotors(int m1, int m2, int m3, int m4) {
  setOneMotor(0, m1); // Front Left  (M1)
  setOneMotor(1, m2); // Front Right (M2)
  setOneMotor(2, m3); // Rear Left   (M3)
  setOneMotor(3, m4); // Rear Right  (M4)
}

// ================== CÁC KIỂU DI CHUYỂN CƠ BẢN ==================

// Dừng tự do: tắt PWM, tắt hết IN → bánh quay trơn (không phanh)
void stopFree() {
  setAllMotors(0, 0, 0, 0);
}

// Phanh cứng: đưa cả 2 IN của từng motor lên HIGH, PWM=0
// → cuộn dây motor bị ngắn mạch, tạo moment phanh
void brakeHard() {
  // M1
  digitalWrite(IN11, HIGH);
  digitalWrite(IN21, HIGH);
  analogWrite(ENA1, 0);

  // M2
  digitalWrite(IN31, HIGH);
  digitalWrite(IN41, HIGH);
  analogWrite(ENB1, 0);

  // M3
  digitalWrite(IN12, HIGH);
  digitalWrite(IN22, HIGH);
  analogWrite(ENA2, 0);

  // M4
  digitalWrite(IN32, HIGH);
  digitalWrite(IN42, HIGH);
  analogWrite(ENB2, 0);
}

// Tiến thẳng: tất cả bánh quay cùng chiều, cùng tốc độ
void goForward(uint8_t speed) {
  setAllMotors(speed, speed, speed, speed);
}

// Lùi thẳng: tất cả bánh quay ngược chiều
void goBackward(uint8_t speed) {
  setAllMotors(-speed, -speed, -speed, -speed);
}

// Quay trái tại chỗ: bánh bên trái lùi, bên phải tiến
void spinLeft(uint8_t speed) {
  setAllMotors(-speed, speed, -speed, speed);
}

// Quay phải tại chỗ: bánh bên trái tiến, bên phải lùi
void spinRight(uint8_t speed) {
  setAllMotors(speed, -speed, speed, -speed);
}

// ================== RẼ MỀM (G, I, H, J) ==================
// Ý tưởng: bên ngoài cua tốc độ = outer, bên trong cua = inner (outer * diagFactor)

// G: Tiến + lệch trái (tiến nhưng ưu tiên cua sang trái)
void forwardLeft(uint8_t speed) {
  int outer = speed;
  int inner = speed * diagFactor;
  // Bên trái/ phải là trong/ngoài cua phụ thuộc chiều lắp bánh thực tế.
  // Ở đây giả sử bên trái là ngoài cua → nhanh hơn.
  setAllMotors(outer, inner, outer, inner);
}

// I: Tiến + lệch phải
void forwardRight(uint8_t speed) {
  int outer = speed;
  int inner = speed * diagFactor;
  setAllMotors(inner, outer, inner, outer);
}

// H: Lùi + lệch trái
void backwardLeft(uint8_t speed) {
  int outer = -speed;
  int inner = -(speed * diagFactor);
  setAllMotors(outer, inner, outer, inner);
}

// J: Lùi + lệch phải
void backwardRight(uint8_t speed) {
  int outer = -speed;
  int inner = -(speed * diagFactor);
  setAllMotors(inner, outer, inner, outer);
}

// ================== ĐI NGANG MECANUM (STRAFE) ==================
// Nếu đi ngược hướng mong muốn, chỉ cần đảo dấu (+/-) trong pattern.

// Q / q: đi ngang sang TRÁI (strafe left)
// Pattern chuẩn Mecanum cho vy < 0: FL(+), FR(-), RL(-), RR(+)
void strafeLeft(uint8_t speed) {
  int v = speed;
  setAllMotors(+v, -v, -v, +v);
}

// E / e: đi ngang sang PHẢI (strafe right)
// Pattern chuẩn Mecanum cho vy > 0: FL(-), FR(+), RL(+), RR(-)
void strafeRight(uint8_t speed) {
  int v = speed;
  setAllMotors(-v, +v, +v, -v);
}

// ================== XỬ LÝ LỆNH TỪ BLUETOOTH (HC-06) ==================
void handleCommand(char cmd) {
  // --------- NHÓM PHÍM SỐ 0..9: CHỌN MỨC TỐC ĐỘ ----------

  if (cmd >= '0' && cmd <= '9') {
    int level = cmd - '0'; // level nằm trong [0..9]

    // Map 0..9 → SPEED_MIN..SPEED_MAX
    // 0 → SPEED_MIN, 9 → SPEED_MAX, các giá trị giữa nội suy tuyến tính
    currentSpeed = SPEED_MIN + (int)((SPEED_MAX - SPEED_MIN) * (float)level / 9.0f);

    Serial.print(F("[CMD] SPEED LEVEL "));
    Serial.print(level);
    Serial.print(F(" -> "));
    Serial.println(currentSpeed);
    return; // Đã xử lý xong, không cần vào switch bên dưới
  }

  // --------- NHÓM LỆNH CHỮ CÁI VÀ KÝ TỰ ĐẶC BIỆT ----------
  switch (cmd) {

    // ===== DI CHUYỂN CƠ BẢN =====
    case 'F': case 'f':
      goForward(currentSpeed);
      Serial.println(F("[CMD] FORWARD"));
      break;

    case 'B': case 'b':
      goBackward(currentSpeed);
      Serial.println(F("[CMD] BACKWARD"));
      break;

    case 'L': case 'l':
      spinLeft(currentSpeed);
      Serial.println(F("[CMD] SPIN LEFT"));
      break;

    case 'R': case 'r':
      spinRight(currentSpeed);
      Serial.println(F("[CMD] SPIN RIGHT"));
      break;

    // ===== DỪNG / PHANH =====
    case 'S': case 's':
      stopFree();
      Serial.println(F("[CMD] STOP FREE"));
      break;

    case 'P': case 'p':
      brakeHard();
      Serial.println(F("[CMD] BRAKE HARD"));
      break;

    // ===== TĂNG / GIẢM TỐC TỪ TỪ =====
    case '+':
      currentSpeed += SPEED_STEP;
      if (currentSpeed > SPEED_MAX) currentSpeed = SPEED_MAX;
      Serial.print(F("[CMD] SPEED UP -> "));
      Serial.println(currentSpeed);
      break;

    case '-':
      currentSpeed -= SPEED_STEP;
      if (currentSpeed < SPEED_MIN) currentSpeed = SPEED_MIN;
      Serial.print(F("[CMD] SPEED DOWN -> "));
      Serial.println(currentSpeed);
      break;

    // ===== RẼ MỀM (G, I, H, J) =====
    case 'G': case 'g':
      forwardLeft(currentSpeed);
      Serial.println(F("[CMD] FORWARD-LEFT (G)"));
      break;

    case 'I': case 'i':
      forwardRight(currentSpeed);
      Serial.println(F("[CMD] FORWARD-RIGHT (I)"));
      break;

    case 'H': case 'h':
      backwardLeft(currentSpeed);
      Serial.println(F("[CMD] BACKWARD-LEFT (H)"));
      break;

    case 'J': case 'j':
      backwardRight(currentSpeed);
      Serial.println(F("[CMD] BACKWARD-RIGHT (J)"));
      break;

    // ===== ĐI NGANG MECANUM (Q, E) =====
    case 'Q': case 'q':
      strafeLeft(currentSpeed);
      Serial.println(F("[CMD] STRAFE LEFT (Q)"));
      break;

    case 'E': case 'e':
      strafeRight(currentSpeed);
      Serial.println(F("[CMD] STRAFE RIGHT (E)"));
      break;

    // ===== LỆNH KHÔNG HỢP LỆ =====
    default:
      Serial.print(F("[CMD] UNKNOWN: "));
      Serial.println(cmd);
      break;
  }
}

// ================== HÀM setup() & loop() ==================
void setup() {
  // Serial (USB) dùng để debug trên Serial Monitor
  Serial.begin(115200);

  // Serial3 – UART3 của Arduino Mega, dùng để nối với HC-06
  // Mặc định HC-06 dùng baud 9600. Nếu bạn đã cấu hình lại HC-06 thì chỉnh baud cho phù hợp.
  Serial3.begin(9600);

  initMotorPins();

  Serial.println(F("=== Mecanum Robot + 2x L298N + HC06 READY ==="));
  Serial.print(F("Current speed = ")); Serial.println(currentSpeed);
  Serial.print(F("diagFactor = "));   Serial.println(diagFactor);
}

void loop() {
  // Nếu có dữ liệu từ Bluetooth (HC-06) gửi sang
  if (Serial3.available()) {
    char c = Serial3.read();   // Đọc 1 ký tự lệnh
    handleCommand(c);          // Xử lý lệnh
  }

  // Có thể thêm code khác ở đây (chớp LED, watchdog, ... nếu cần)
}
