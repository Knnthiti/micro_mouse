#define IR_L_PIN 17
#define IR_LF_PIN 16
#define IR_F_PIN 15
#define IR_RF_PIN 1
#define IR_R_PIN 41

#define REC_L_PIN 7
#define REC_LF_PIN 6
#define REC_F_PIN 5
#define REC_RF_PIN 4
#define REC_R_PIN 2

#define MOTOR_LA 12
#define MOTOR_LB 13
#define MOTOR_RA 14
#define MOTOR_RB 21

#define ENC1A_PIN 10
#define ENC1B_PIN 11
#define ENC2A_PIN 47
#define ENC2B_PIN 48

#define BUT1_PIN 18
#define BUZZER_PIN 45
#define EN_PIN 0

#include <ESP32Encoder.h>

ESP32Encoder enc1;
ESP32Encoder enc2;

///////////////////////////////////////////////Class speed(หาความเร็ว)///////////////////////////////////////////////////////////
//ตัวแปรหาค่าความเร็ว motor
unsigned long t = 0;
int e;                    //Encoder ตัวที่ e
long past[2] = { 0, 0 };  //ค่า Encoder อดีต
long present[2];          //ค่า Encoder ปัจจุบัน
double speed[2];          //{speed_L,speed_R}   //ความเร็ว Encoder

double Point_EN[2] = { 0, 0 };  //ระยะ
long Encoder100_ms[2] = { 0, 0 };

#define LOOP_TIME_MS 1                      // 1ms
#define LOOP_FREQUENCY 1.0f / LOOP_TIME_MS  //ความถี่(1/ms)

void getEn_ms(int e) {  //หาค่าความเร็ว motor (Encoder*100 / ms)
  long enc_Count[2] = { enc1.getCount(), enc2.getCount() };
  present[e] = enc_Count[e];                           //รับค่า Encoder
  speed[e] = (present[e] - past[e]) / LOOP_TIME_MS;    //ค่า Encoder / ms
  Point_EN[e] = Point_EN[e] + (present[e] - past[e]);  //ระยะทาง
  past[e] = present[e];                                //บันทึกค่าปัจจุบัน
  Encoder100_ms[e] = speed[e] * 100;                   //แปลงให้ความเร็ว Encoder/ms ใช้ง่ายขึ้น
}

////////////////////////////////////////////////////////Class point////////////////////////////////////////////////////////////////
//ตัวแปรสำหรับ PID point
#define length_1_box 500
long length_L;
long length_R;
long length;
long Set_Slow;
long error_length;  //ค่าerror

int En100_ms[2];

#define min_En100_ms 0
#define start_En100_ms 100
#define max_En100_ms 600

void point(int B) {  // B = ตำแหน่ง Box
  //ดักค่า ระยะทาง_L
  if (Point_EN[0] < 0) {
    length_L = -Point_EN[0];
  } else {
    length_L = Point_EN[0];
  }
  //ดักค่า ระยะทาง_R
  if (Point_EN[1] < 0) {
    length_R = -Point_EN[1];
  } else {
    length_R = Point_EN[1];
  }

  length = (length_L + length_R) / 2;  //ระยะทางเฉลี่ย

  Set_Slow = (B - 1) * (length_1_box / 2);  //ตำแหน่งเริ่ม slow
  if (length >= Set_Slow) {                 //Slow
    error_length = length_1_box - (length - Set_Slow);
    if (error_length > 0) {
      En100_ms[0] = map(error_length, length_1_box, 0, max_En100_ms, min_En100_ms);
      En100_ms[1] = map(error_length, length_1_box, 0, max_En100_ms, min_En100_ms);

      if (En100_ms[0] < start_En100_ms) {
        En100_ms[0] = start_En100_ms;
      }
      if (En100_ms[1] < start_En100_ms) {
        En100_ms[1] = start_En100_ms;
      }

      Lmotor(En100_ms[0]);
      Rmotor(En100_ms[1]);
    } else {
      Lmotor(0);
      Rmotor(0);
    }

  } else {                                  //Up_speed
    error_length = (length_1_box / 2) - length;
    if (error_length >= 0) {
      En100_ms[0] = map(error_length, length_1_box, 0, min_En100_ms, max_En100_ms);
      En100_ms[1] = map(error_length, length_1_box, 0, min_En100_ms, max_En100_ms);

      if (En100_ms[0] < start_En100_ms) {
        En100_ms[0] = start_En100_ms;
      }
      if (En100_ms[1] < start_En100_ms) {
        En100_ms[1] = start_En100_ms;
      }

      Lmotor(En100_ms[0]);
      Rmotor(En100_ms[1]);
    } else {
      Lmotor(max_En100_ms);
      Rmotor(max_En100_ms);
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////Class IR sensor////////////////////////////////////////////////////////
int IR_LED_PIN[5] = { IR_L_PIN, IR_LF_PIN, IR_F_PIN, IR_RF_PIN, IR_R_PIN };
int IR_REC_PIN[5] = { REC_L_PIN, REC_LF_PIN, REC_F_PIN, REC_RF_PIN, REC_R_PIN };

//ตัวแปร กระเด้าซ้าย&กระเด้าขวา
long   IR_read[5];
long   IR_max[5]      = { 3730, 0, 3780, 0, 3630 };    //เข้าใกล้กำแพงมาก
long   IR_min[5]      = {  830, 0, 1220, 0, 900  };    //เข้าใกล้กำแพงน้อย
long   setpoint_IR[5] = { 0.65f, 0.0f, 0.0f, 0.0f, 0.75f };

//ตัวแปร PID กระเด้าซ้ายและกระเด้าขวา
double error_IR[5];                                          // {error_IR_L        ,0,0,0, error_IR_R}
double past_error_IR[5];                                     // {past_error_IR_L   ,0,0,0, past_error_IR_R}
double point_IR[5];                                          // {point_IR_L        ,0,0,0, point_IR_R}
#define point_max 1.0f;
double proportinal_IR[5];                                    // {proportinal_IR_L ,0,0,0, proportinal_IR_R}
double integnator_IR[5];                                     // {integnator_IR_L  ,0,0,0, integnator_IR_R}
double derivative_IR[5];                                     // {derivative_IR_L  ,0,0,0, derivative_IR_R}
double output_PID_IR[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };  // {output_PID_IR_L  ,0,0,0, output_PID_IR_R} ใช้เพิ่มค่าความเร็ว motor จาก IR


#define Kp_IR_L 3.5f
#define Ki_IR_L 0.3f
#define Kd_IR_L 0.5f

#define Kp_IR_R 3.5f
#define Ki_IR_R 0.3f
#define Kd_IR_R 0.5f

double Kp_IR[5] = { Kp_IR_L, 0, 0, 0, Kp_IR_R };
double Ki_IR[5] = { Ki_IR_L, 0, 0, 0, Ki_IR_R };
double Kd_IR[5] = { Kd_IR_L, 0, 0, 0, Kd_IR_R };


//PIDเคลื่อนที่โดยIR
void move_IR(int r) {
  if (IRread(r) > IR_min[r]) {  //ตรวจกำแพงซ้าย,ขวา,หน้า
    if (r == 0) {               //ถ้าเจอกำแพงซ้าย ตรวจ ขวา,หน้า
      if (IRread(4) > IR_min[4]) {
        if (IRread(2) > IR_min[2]) {  //หากเจอกำแพงซ้าย,ขวา,หน้า
          Stop();
        }
      }
    }
    if (r == 4) {  //ถ้าเจอกำแพงขวา ตรวจ ซ้าย,หน้า
      if (IRread(0) > IR_min[0]) {
        if (IRread(2) > IR_min[2]) {  //หากเจอกำแพงซ้าย,ขวา,หน้า
          Stop();
        }
      }
    }
    IR_read[r] = IRread(r);
    point_IR[r] = (IR_read[r] - IR_min[r]) / (IR_max[r] - IR_min[r]);  //ตำแหน่งหุ่น ชิดซ้ายสุดมีค่า=0 ชิดขวาสุดมีค่า=1
    //ดักค่าเกิน
    if (point_IR[r] > 1.0F) { point_IR[r] = 1.0f; }
    if (point_IR[r] < 0.0F) { point_IR[r] = 0.0f; }
    //PID_IR_L
    error_IR[r] = setpoint_IR[r] - point_IR[r];
    proportinal_IR[r] = error_IR[r] * (Kp_IR[r]);
    integnator_IR[r] = (error_IR[r] + past_error_IR[r]) * (Ki_IR[r]);
    derivative_IR[r] = (error_IR[r] - past_error_IR[r]) * (Kd_IR[r]);
    past_error_IR[r] = error_IR[r];
    output_PID_IR[r] = proportinal_IR[r] + integnator_IR[r] + derivative_IR[r];
    output_PID_IR[r] = -output_PID_IR[r];
    neopixelWrite(38, 0 * 20, 0 * 20, 0);
  } else {
    if (r == 0) {  //ตรวจกำแพงซ้ายไม่เจอ
      neopixelWrite(38, 10, 0, 0);
      Serial.println("Turn left");
      Turn_left();
    }
    if (r == 4) {  //ตรวจกำแพงขวาไม่เจอ
      neopixelWrite(38, 0, 0, 10);
      Serial.println("Turn Right");
      Turn_right();
    }
  }
}

//IRread=อ่านค่าIRเซนเซอร์
double IRread(int ch) {
  double ret = 0;
  digitalWrite(IR_LED_PIN[ch], HIGH);
  delayMicroseconds(10);
  ret = analogRead(IR_REC_PIN[ch]);
  digitalWrite(IR_LED_PIN[ch], LOW);
  return ret;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////Class Motor///////////////////////////////////////////////////////////
//ตัวแปรสำหรับ PID motor
double error[2];  //ค่าerror
double proportinal[2];
double integnator[2];
double derivative[2];
double past_error[2] = { 0, 0 };
double output_PID[2];

#define Kp_L 5.0f
#define Ki_L 0.3f
#define Kd_L 1.0f

#define Kp_R 5.0f
#define Ki_R 0.3f
#define Kd_R 1.0f


long duty_cycle_L;
long duty_cycle_R;

#define min_duty_cycle_L 1800
#define max_duty_cycle_L 3000

#define min_duty_cycle_R 1800
#define max_duty_cycle_R 3000

int output_PID_IR_L;  //ตัวแปรเก็บค่า PID IR ที่มาพิจารณา
int output_PID_IR_R;  //ตัวแปรเก็บค่า PID IR ที่มาพิจารณา

//สั่งmotorด้วยPID
void Lmotor(int i) {
  output_PID_IR_L = output_PID_IR[0] * 100;  //เอาค่า PID IR มาพิจารณา
  i = i + output_PID_IR_L;
  //PID_motor_L
  error[0] = i - Encoder100_ms[0];
  proportinal[0] = error[0] * Kp_L;
  integnator[0] = (error[0] + past_error[0]) * Ki_L;
  derivative[0] = (error[0] - past_error[0]) * Kd_L;
  past_error[0] = error[0];
  output_PID[0] = proportinal[0] + integnator[0] + derivative[0];

  //output_PID(En_ms) to duty_cycle
  duty_cycle_L = map(output_PID[0], min_En100_ms, max_En100_ms, min_duty_cycle_L, max_duty_cycle_L);

  //ดักค่า
  if (duty_cycle_L > max_duty_cycle_L) {
    duty_cycle_L = max_duty_cycle_L;
  }
  ledcWrite(0, 0);
  ledcWrite(1, duty_cycle_L);
  // ledcWrite(1, 0);
}

void Rmotor(int j) {
  Encoder100_ms[1] = -Encoder100_ms[1];      //เนื่องจากค่า Encoder_motor_R ติดลบ จึงแปลงเป็นบวก
  output_PID_IR_R = output_PID_IR[4] * 100;  //เอาค่า PID IR มาพิจารณา
  j = j + output_PID_IR_R;
  //PID_motor_R
  error[1] = j - Encoder100_ms[1];
  proportinal[1] = error[1] * Kp_R;
  integnator[1] = (error[1] + past_error[1]) * Ki_R;
  derivative[1] = (error[1] - past_error[1]) * Kd_R;
  past_error[1] = error[1];
  output_PID[1] = proportinal[1] + integnator[1] + derivative[1];

  //output_PID(RPM) to duty_cycle
  duty_cycle_R = map(output_PID[1], min_En100_ms, max_En100_ms, min_duty_cycle_R, max_duty_cycle_R);
  
  //ดักค่า
  if (duty_cycle_R > max_duty_cycle_R) {
    duty_cycle_R = max_duty_cycle_R;
  }
  ledcWrite(2, duty_cycle_R);
  // ledcWrite(2, 0);
  ledcWrite(3, 0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////Class Move/////////////////////////////////////////////////////
#define Front_before_Turn  110          //ไปข้างหน้านิดหนึ่ง
#define Length_Turn_L_90   211          //หันซ้าย 90 องศา
#define Length_Turn_R_90   210          //หันขวา 90 องศา
#define Front_after_Turn_R 265          //ไปข้างหน้านิดหนึ่งเมื่อหันขวาเสร็จ
#define Front_after_Turn_L 322          //ไปข้างหน้านิดหนึ่งเมื่อหันซ้ายเสร็จ

long Save_Point_EN[2];


void Turn_left() {                  //เลี้ยวซ้าย
  Save_Point_EN[1] = abs(enc2.getCount());        //บันทึก Encoder ตอนเริ่ม  [abs(x) >> แปลง x เป็นค่าสมบูรณ์]

  while ((abs(enc2.getCount()) - Save_Point_EN[1]) < Front_before_Turn) {   //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 0);
    ledcWrite(1, 2000);
    ledcWrite(2, 2000);
    ledcWrite(3, 0);
  }

  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);


  Save_Point_EN[1] = abs(enc2.getCount());  //บันทึก Encoder ตอนเริ่ม

  while ((abs(enc2.getCount()) - Save_Point_EN[1]) < Length_Turn_L_90) {    //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 2600);
    ledcWrite(1, 0);
    ledcWrite(2, 2600);
    ledcWrite(3, 0);
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);

  delay(5);

  Save_Point_EN[1] = abs(enc2.getCount());  //บันทึก Encoder ตอนเริ่ม

  while ((abs(enc2.getCount()) - Save_Point_EN[1]) < Front_after_Turn_L) {  //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 0);
    ledcWrite(1, 2000);
    ledcWrite(2, 2000);
    ledcWrite(3, 0);
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void Turn_right() {                        //เลี้ยวขวา
  Save_Point_EN[0] = abs(enc1.getCount());  //บันทึก Encoder ตอนเริ่ม

  while ((abs(enc1.getCount()) - Save_Point_EN[0]) < Front_before_Turn) {  //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 0);
    ledcWrite(1, 2000);
    ledcWrite(2, 2000);
    ledcWrite(3, 0);
  }

  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);


  Save_Point_EN[0] = abs(enc1.getCount());  //บันทึก Encoder ตอนเริ่ม

  while ((abs(enc1.getCount()) - Save_Point_EN[0]) < Length_Turn_R_90) {  //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 0);
    ledcWrite(1, 2600);
    ledcWrite(2, 0);
    ledcWrite(3, 2600);
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);

  delay(5);

  Save_Point_EN[0] = abs(enc1.getCount());  //บันทึก Encoder ตอนเริ่ม

  while ((abs(enc1.getCount()) - Save_Point_EN[0]) < Front_after_Turn_R) {  //(ผลต่างระยะทาง(Encoder(ปัจจุบัน)-Encoder(ตอนเริ่ม)) < ระยะทางที่กำหนด)
    ledcWrite(0, 0);
    ledcWrite(1, 1800);
    ledcWrite(2, 1800);
    ledcWrite(3, 0);
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void Stop() {              //หยุดหุ่น
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  delay(125);
}

void Turn_back() {

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(38, OUTPUT);
  pinMode(BUT1_PIN, INPUT_PULLUP);
  pinMode(EN_PIN, INPUT);  // external pullup
  neopixelWrite(38, 0, 0, 0);
  
  //on-off
  pinMode(BUT1_PIN, INPUT_PULLUP);
  while (digitalRead(BUT1_PIN) == HIGH) {
    delay(20);
  }
  Serial.begin(115200);

  //ประกาศ pin output ของ IR
  pinMode(IR_L_PIN, OUTPUT);
  pinMode(IR_LF_PIN, OUTPUT);
  pinMode(IR_F_PIN, OUTPUT);
  pinMode(IR_RF_PIN, OUTPUT);
  pinMode(IR_R_PIN, OUTPUT);

  pinMode(REC_L_PIN, INPUT);
  pinMode(REC_LF_PIN, INPUT);
  pinMode(REC_F_PIN, INPUT);
  pinMode(REC_RF_PIN, INPUT);
  pinMode(REC_R_PIN, INPUT);

  digitalWrite(IR_L_PIN, LOW);
  digitalWrite(IR_LF_PIN, LOW);
  digitalWrite(IR_F_PIN, LOW);
  digitalWrite(IR_RF_PIN, LOW);
  digitalWrite(IR_R_PIN, LOW);

  //ประกาศ pin output ของ Encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  enc1.attachHalfQuad(ENC1A_PIN, ENC1B_PIN);
  enc1.clearCount();

  enc2.attachHalfQuad(ENC2A_PIN, ENC2B_PIN);
  enc2.clearCount();

  //ประกาศ pin output ของ motor
  ledcSetup(0, 2000, 12);
  ledcSetup(1, 2000, 12);
  ledcSetup(2, 2000, 12);
  ledcSetup(3, 2000, 12);

  ledcAttachPin(MOTOR_LA, 0);
  ledcAttachPin(MOTOR_LB, 1);
  ledcAttachPin(MOTOR_RA, 2);
  ledcAttachPin(MOTOR_RB, 3);

  Lmotor(0);
  Rmotor(0);
}

void loop() {
  if ((millis() - t) > LOOP_TIME_MS) {
    t = millis();
    //หาความเร็ว
    getEn_ms(0);
    getEn_ms(1);

    //เย็ดซ้าย-ขวา
    move_IR(0);
    move_IR(4);
    
    //แสดงค่า Encoder ที่นับได้
    Serial.print(" L = ");
    Serial.print(enc1.getCount());
    Serial.print(" R = ");
    Serial.println(enc2.getCount());

    //แสดงตำแหน่งหุ่นจากกำแพงซ้าย-ขวา
    // Serial.print("point_IR_L = ");
    // Serial.print(point_IR[0]);
    // Serial.print(", point_IR_R = ");
    // Serial.println(point_IR[4]);

    //แสดงค่าที่เซนเซอร์ IR อ่านได้
    // Serial.print(" L = ");
    // Serial.print(IRread(0));
    // Serial.print(" L_F = ");
    // Serial.print(IRread(1));
    // Serial.print(", F = ");
    // Serial.print(IRread(2));
    // Serial.print(", R_F = ");
    // Serial.print(IRread(3));
    // Serial.print(" R = ");
    // Serial.println(IRread(4));

    //แสดง PID ของ IR
    // Serial.print("PID_IR_L = ");
    // Serial.print(output_PID_IR[0]);
    // Serial.print(", PID_IR_R = ");
    // Serial.println(output_PID_IR[4]);

    //แสดง ความเร็วล้อ
    // Serial.print("speed_L = ");
    // Serial.print(speed[0]);
    // Serial.print(" ,speed_R = ");
    // Serial.println(speed[1]);

    // point(10);  //เคลื่อนที่ไป 10 Box

    Lmotor(300);  //สั่ง motor_L ทำงาน (0-600)
    Rmotor(300);  //สั่ง motor_R ทำงาน (0-600)

  }
}