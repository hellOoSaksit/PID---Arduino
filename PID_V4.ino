//เรียกใช้ BL ของ Servo
#include <Servo.h>
// กำหนดขาสำหรับ TCRT5000 ไม่ใช้ขา 9 และ 10 
#define out5 ???
#define out4 ???
#define out3 ???
#define out2 ???
#define out1 ???
//กำหนดขาสำหรับ L9110S ต้องเป็นค่า PWM (3, 5, 6, 9, 10, 11) ไม่ใครใช้ ขา 9 และ 10
#define B1A ???
#define B1B ???
#define A1A ???
#define A1B ???
// กำหนดตัวแปร PID & ตั้งค่าระบบ แก้ไขแค่ตรงนี้ เพือปรับประสิทธ์ภาพรถ
double Kp = 30; //การเข้าโค้ง
double Ki = 0;
double Kd = 0;
int Setpoint = 80; //ความเร็ว

double error = 0;
double stop = 0;
double last_error = 0;
double integral = 0;
double derivative = 0;
double pid_value = 0;

// กำหน่วยช่วง ARM ทำงาน
int ArmSetPoint = 0;
int ArmSet_UpDown = 0;
int ArmServoR_old, ArmServoL_old, ArmServoBase_old = 0;
// กำหนดตัวแปร ตัวจับเส้น
int S1 = 0, S2 = 0, S3 = 0, S4 = 0, S5 = 0;
// Servo
Servo ArmServoR, ArmServoL, ArmServoBase;

void setup() {

  // กำหนดขาเป็น INPUT
  pinMode(out1, INPUT);
  pinMode(out2, INPUT);
  pinMode(out3, INPUT);
  pinMode(out4, INPUT);
  pinMode(out5, INPUT);
  pinMode(B1A, OUTPUT);
  pinMode(B1B, OUTPUT);
  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);

//  ArmServoR.attach(9);
  ArmServoL.attach(10);
  ArmServoBase.attach(13);
  ArmServoBase.write(0);
  ArmServoL.write(0);
  for(int pos = 0; pos <= 90; pos += 1) { // คำสั่ง for loop เพื่อทำให้ Servo หมุนไปจนถึง 180 องศา
    ArmServoBase.write(pos); // ส่งค่า pos ไปยัง Servo
    delay(15); // หน่วงเวลา 15 มิลลิวินาที
  }
  // เริ่มต้น Serial Communication
  Serial.begin(9600);
}

void loop() {
  // อ่านค่า analog จาก TCRT5000 แต่ละขา
  S1 = digitalRead(out1);
  S2 = digitalRead(out2);
  S3 = digitalRead(out3);
  S4 = digitalRead(out4);
  S5 = digitalRead(out5);

  // คำนวณค่า error โดยใช้ตารางข้อมูล
  if (S1 == 1 and S2 == 1 and S3 == 1 and S4 == 1 and S5 == 0) {
    error = 4;
  } else if (S1 == 1 and S2 == 1 and S3 == 1 and S4 == 0 and S5 == 0) {
    error = 3;
  } else if (S1 == 1 and S2 == 1 and S3 == 1 and S4 == 0 and S5 == 1) {
    error = 2;
  } else if (S1 == 1 and S2 == 1 and S3 == 0 and S4 == 0 and S5 == 1) {
    error = 1;
  } else if (S1 == 1 and S2 == 1 and S3 == 0 and S4 == 1 and S5 == 1) {
    error = 0;
  } else if (S1 == 1 and S2 == 0 and S3 == 0 and S4 == 1 and S5 == 1) {
    error = -1;
  } else if (S1 == 1 and S2 == 0 and S3 == 1 and S4 == 1 and S5 == 1) {
    error = -2;
  } else if (S1 == 0 and S2 == 0 and S3 == 1 and S4 == 1 and S5 == 1) {
    error = -3;
  } else if (S1 == 0 and S2 == 1 and S3 == 1 and S4 == 1 and S5 == 1) {
    error = -4;
  } else if (S1 == 0 and S2 == 0 and S3 == 0 and S4 == 0 and S5 == 0) {
    ArmSetPoint = 1;
  } else if (S1 == 1 and S2 == 1 and S3 == 1 and S4 == 1 and S5 == 1) {
  }
  // คำนวณค่า PID
  integral = integral + error;
  derivative = error - last_error;
  pid_value = Kp * error + Ki * integral + Kd * derivative;

  // บันทึกค่า error ล่าสุด
  last_error = error;

  // คำนวณค่า PWM ที่จะส่งไปยังมอเตอร์
  int motorSpeedA = constrain(Setpoint + pid_value, 0, 1023);
  int motorSpeedB = constrain(Setpoint - pid_value, 0, 1023);
  if (ArmSetPoint == 1) {
    digitalWrite(B1A, HIGH);
    digitalWrite(B1B, HIGH);
    digitalWrite(A1A, HIGH);
    digitalWrite(A1B, HIGH);
    delay(500);
    //ฟังชั่น แขนกล
    if (ArmSet_UpDown == 0) {
      //ตอนจับของ
      for(int pos = 90; pos >= 8; pos -= 1) { // คำสั่ง for loop เพื่อทำให้ Servo หมุนกลับไปที่ 0 องศา
        ArmServoBase.write(pos); // ส่งค่า pos ไปยัง Servo
        delay(15); // หน่วงเวลา 15 มิลลิวินาที
      }
      delay(500);
      for(int pos = 0; pos <= 90; pos += 1) { // คำสั่ง for loop เพื่อทำให้ Servo หมุนไปจนถึง 180 องศา
        ArmServoL.write(pos); // ส่งค่า pos ไปยัง Servo
        delay(15); // หน่วงเวลา 15 มิลลิวินาที
      }
      ArmSet_UpDown = 1;
    } else if (ArmSet_UpDown == 1) {
      //ตอนวางของ
      for(int pos = 0; pos <= 90; pos += 1) { // คำสั่ง for loop เพื่อทำให้ Servo หมุนไปจนถึง 180 องศา
        ArmServoBase.write(pos); // ส่งค่า pos ไปยัง Servo
        delay(15); // หน่วงเวลา 15 มิลลิวินาที
      }
      delay(500);
      for(int pos = 90; pos >= 0; pos -= 1) { // คำสั่ง for loop เพื่อทำให้ Servo หมุนกลับไปที่ 0 องศา
        ArmServoL.write(pos); // ส่งค่า pos ไปยัง Servo
        delay(15); // หน่วงเวลา 15 มิลลิวินาที
      }

      ArmSet_UpDown = 0;
    }
    //รีเซตจุดวิ่งไปข้างหน้านึงหนึง
    ArmSetPoint = 0;
    digitalWrite(B1A, LOW);
    analogWrite(B1B, Setpoint);
    digitalWrite(A1A, LOW);
    analogWrite(A1B, Setpoint);
    delay(500);
  } else {
    // ส่งค่า PWM ไปยังมอเตอร์
    digitalWrite(B1A, LOW);
    analogWrite(B1B, motorSpeedB);
    digitalWrite(A1A, LOW);
    analogWrite(A1B, motorSpeedA);
  }

}
