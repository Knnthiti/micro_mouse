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

///////////////////////////////////////////////speed(หาความเร็ว)///////////////////////////////////////////////////////////
//ตัวแปรหาค่าความเร็ว motor
unsigned long t=0;
int e ;                               //Encoder ตัวที่ e
long past[2];                        //ค่า Encoder อดีต
long present[2] ;                    //ค่า Encoder ปัจจุบัน
double speed[2] ;//{speed_L,speed_R}   //ความเร็ว Encoder

double Point_EN[2] = {0,0};  //ระยะ

#define LOOP_TIME_MS    1 // 1ms
#define LOOP_FREQUENCY  1.0f/LOOP_TIME_MS           //ความถี่(1/ms)       

void getEn_ms(int e){                                  //หาค่าความเร็ว motor (Encoder / ms) 
  long enc_Count[2] = {enc1.getCount(),enc2.getCount()};
  present[e] = enc_Count[e];                           //รับค่า Encoder
  speed[e] = (present[e]-past[e]) / LOOP_TIME_MS;     //ค่า Encoder / ms 
  Point_EN[e]=Point_EN[e]+(present[e]-past[e]);       //ระยะทาง
  past[e] = present[e];                               //บันทึกค่าปัจจุบัน
}

////////////////////////////////////////////////////////point////////////////////////////////////////////////////////////////
//ตัวแปรสำหรับ PID point
#define length_1_box 500
long length_L;
long length_R;
long length;
long Set_Slow;
long error_length;                //ค่าerror

int duty_cycle[2];
#define min_duty_cycle_L 2000
#define max_duty_cycle_L 4000

#define min_duty_cycle_R 2000
#define max_duty_cycle_R 4000



void point(int B){    // B = ตำแหน่ง Box
  //ดักค่า ระยะทาง_L
  if(Point_EN[0]<0){
    length_L=-Point_EN[0];
  }else{
    length_L=Point_EN[0];
  }
  //ดักค่า ระยะทาง_R
  if(Point_EN[1]<0){
    length_R=-Point_EN[1];
  }else{
    length_R=Point_EN[1];
  }

  length = (length_L+length_R)/2;  //ระยะทางเฉลี่ย

  Set_Slow=(B-1)*length_1_box;     //ตำแหน่งเริ่ม slow
  if(length>=Set_Slow){                        //Slow
   error_length=length_1_box-(length-Set_Slow);
   if(error_length>0){
     duty_cycle[0] = map(error_length,length_1_box,0,max_duty_cycle_L,min_duty_cycle_L);
     duty_cycle[1] = map(error_length,length_1_box,0,max_duty_cycle_R,min_duty_cycle_R);
    // duty_cycle[0] = max_duty_cycle_R-((error_length/length_1_box)*max_duty_cycle_L);
    // duty_cycle[1] = max_duty_cycle_R-((error_length/length_1_box)*max_duty_cycle_R);
     Lmotor(duty_cycle[0]);
     Rmotor(duty_cycle[1]);
   }else{
    //  Lmotor(min_duty_cycle_L);
    //  Rmotor(min_duty_cycle_R);
     Lmotor(0);
     Rmotor(0);
   }

  }else{                                     //Up_speed
    error_length=length_1_box-length;
    if(error_length>=0){
      duty_cycle[0] = map(error_length,length_1_box,0,min_duty_cycle_L,max_duty_cycle_L);
      duty_cycle[1] = map(error_length,length_1_box,0,min_duty_cycle_R,max_duty_cycle_R);
      // duty_cycle[0] = ((error_length/length_1_box)*max_duty_cycle_L)+min_duty_cycle_L;
      // duty_cycle[1] = ((error_length/length_1_box)*max_duty_cycle_R)+min_duty_cycle_R;
      Lmotor(duty_cycle[0]);
      Rmotor(duty_cycle[1]);
    }else{
      Lmotor(max_duty_cycle_L);
      Rmotor(max_duty_cycle_R);
    }

  }

}

//////////////////////////////////////////////////IR sensor////////////////////////////////////////////////////////
int IR_LED_PIN[5] = {IR_L_PIN ,  IR_LF_PIN ,  IR_F_PIN,  IR_RF_PIN,  IR_R_PIN};
int IR_REC_PIN[5] = {REC_L_PIN , REC_LF_PIN , REC_F_PIN, REC_RF_PIN, REC_R_PIN};

//ตัวแปร กระเด้าซ้าย&กระเด้าขวา
int IR_read[5];
float IR_max[5] = {3500,0,0,0,3500};   //เข้าใกล้กำแพงมาก
float IR_min[5] = {850,0,0,0,1430};    //เข้าใกล้กำแพงน้อย
float setpoint_IR[5] = {0.52f,0.0f,0.0f,0.0f,0.45f};

//ตัวแปร PID กระเด้าซ้ายและกระเด้าขวา
double error_IR[5] ;//      {error_IR_L       ,error_IR_R}
double past_error_IR[5] ;// {past_error_IR_L  ,past_error_IR_R}
double point_IR[5];//       {point_IR_L       , point_IR_R}
#define point_max 1.0f;
double proportinal_IR[5] ; // {proportinal_IR_L , proportinal_IR_R}
double integnator_IR[5]  ; // {integnator_IR_L  , integnator_IR_R}
double derivative_IR[5]  ; // {derivative_IR_L  , derivative_IR_R}
double output_PID_IR[5] = {0.0f,0.0f,0.0f,0.0f,0.0f}; // {output_PID_IR_L  , output_PID_IR_R} ใช้เพิ่มค่าความเร็ว motor จาก IR


#define Kp_IR_L 12.0f
#define Ki_IR_L 0.3f
#define Kd_IR_L 1.3f

#define Kp_IR_R 12.0f
#define Ki_IR_R 0.3f
#define Kd_IR_R 1.3f

double Kp_IR[5] = { Kp_IR_L , 0 , 0 , 0 , Kp_IR_R };
double Ki_IR[5] = { Ki_IR_L , 0 , 0 , 0 , Ki_IR_R };
double Kd_IR[5] = { Kd_IR_L , 0 , 0 , 0 , Kd_IR_R };


//PIDเคลื่อนที่โดยIR
void move_IR(int r){
  IR_read[r]=IRread(r);
  if(IRread(r)>IR_min[r]){                                     //ตรวจกำแพงซ้าย,ขวา
  point_IR[r]=(IR_read[r]-IR_min[r])/(IR_max[r]-IR_min[r]);     //ตำแหน่งหุ่น ชิดซ้ายสุดมีค่า=0 ชิดขวาสุดมีค่า=1
  //ดักค่าเกิน
  if(point_IR[r] > 1.0F){ point_IR[r] = 1.0f;}
  if(point_IR[r] < 0.0F){ point_IR[r] = 0.0f;}
  //PID_IR_L
  error_IR[r]=setpoint_IR[r]-point_IR[r];
  proportinal_IR[r]=error_IR[r]*(Kp_IR[r]);
  integnator_IR[r]=(error_IR[r]+past_error_IR[r])*(Ki_IR[r]);
  derivative_IR[r]=(error_IR[r]-past_error_IR[r])*(Kd_IR[r]);
  past_error_IR[r]=error_IR[r];
  output_PID_IR[r]=proportinal_IR[r]+integnator_IR[r]+derivative_IR[r];
  output_PID_IR[r]=-output_PID_IR[r];
    neopixelWrite(38,0* 20,0 * 20, 0 );
  }else{
    if(r==0){
    neopixelWrite(38,1* 20,0 * 20, 0 );
    Turn_left();
    }
    if(r==4){
    neopixelWrite(38,0* 20,1 * 20, 0 );
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

/////////////////////////////////////////motor///////////////////////////////////////////////////////////
//ตัวแปรสำหรับ PID motor
double error[2] ;                //ค่าerror
double proportinal[2] ;
double integnator[2] ;
double derivative[2] ;
double past_error[2] = {0,0};
double output_PID[2];

#define Kp_L 1.0f
#define Ki_L 0.2f
#define Kd_L 0.3f

#define Kp_R 1.0f
#define Ki_R 0.2f
#define Kd_R 0.3f


double duty_cycle_L;
double duty_cycle_R;
#define min_En_ms 0.0f
#define max_En_ms 6.0f

int output_PID_IR_L;  //ตัวแปรเก็บค่า PID IR ที่มาพิจารณา
int output_PID_IR_R;  //ตัวแปรเก็บค่า PID IR ที่มาพิจารณา

//สั่งmotorด้วยPID
void Lmotor(int i) {
  // if(output_PID_IR_L<=0){
  //   output_PID_IR_L=0;
  // }
  output_PID_IR_L=output_PID_IR[0]*300; //เอาค่า PID IR มาพิจารณา
  //i=i+output_PID_IR_L;
  i = map(i,min_duty_cycle_L,max_duty_cycle_L,min_En_ms,max_En_ms); //แปลงจาก duty_cycle to En_ms
  //i = (i/(max_duty_cycle_L-min_duty_cycle_L))*(max_En_ms-min_En_ms);

  //PID_motor_L
  error[0]=i-speed[0];
  proportinal[0]=error[0]*Kp_L;
  integnator[0]=(error[0]-past_error[0])*Ki_L;
  derivative[0]=(error[0]-past_error[0])*Kd_L;
  past_error[0]=error[0];
  output_PID[0]=proportinal[0]+integnator[0]+derivative[0];

  //output_PID(En_ms) to duty_cycle
  //duty_cycle_L = map(output_PID[0],min_En_ms,max_En_ms,min_duty_cycle_L,max_duty_cycle_L);
  duty_cycle_L = ((output_PID[0]/(max_En_ms-min_En_ms))*max_duty_cycle_L)+min_duty_cycle_L;

  //สั่ง motor หมุนด้วยค่า duty_cycle
  duty_cycle_L=duty_cycle_L+output_PID_IR_L; //duty_cycle = output_PID + output_PID_IR
  if(duty_cycle_L > max_duty_cycle_L){
    duty_cycle_L = max_duty_cycle_L;
  }
  // if(duty_cycle_L<min_duty_cycle_L){
  //   duty_cycle_L=0;
  // }
    ledcWrite(0, 0);
    ledcWrite(1, duty_cycle_L);
    //ledcWrite(1, 0);
  Serial.print("duty_cycle_L = ");
  Serial.print(duty_cycle_L);
  
}

void Rmotor(int j) { 
  // if(output_PID_IR_R<=0){
  //   output_PID_IR_R=0;
  // }
  output_PID_IR_R=output_PID_IR[4]*300; //เอาค่า PID IR มาพิจารณา
  //j=j+output_PID_IR_R;
  j = map(j,min_duty_cycle_R,max_duty_cycle_R,min_En_ms,max_En_ms); //แปลงจาก duty_cycle to En_ms
  // j = (j/(max_duty_cycle_R-min_duty_cycle_R))*(max_En_ms-min_En_ms); 

  speed[1]=-speed[1];               //เนื่องจากค่า Encoder_motor_R ติดลบ จึงแปลงเป็นบวก
  //PID_motor_R
  error[1]=j-speed[1];
  proportinal[1]=error[1]*Kp_R;
  integnator[1]=error[1]*Ki_R;
  derivative[1]=(error[1]-past_error[1])*Kd_R;
  past_error[1]=error[1];
  output_PID[1]=proportinal[1]+integnator[1]+derivative[1];

  //output_PID(RPM) to duty_cycle
  duty_cycle_R = map(output_PID[1],min_En_ms,max_En_ms,min_duty_cycle_R,max_duty_cycle_R);
  // duty_cycle_R = ((output_PID[1]/(max_En_ms-min_En_ms))*max_duty_cycle_R)+min_duty_cycle_R;

  //สั่ง motor หมุนด้วยค่า duty_cycle
  duty_cycle_R=duty_cycle_R+output_PID_IR_R; //duty_cycle = output_PID + output_PID_IR
  if(duty_cycle_R > max_duty_cycle_R){
    duty_cycle_R = max_duty_cycle_R;
  }
  // if(duty_cycle_R<min_duty_cycle_R){
  //   duty_cycle_R=0;
  // }
    ledcWrite(2, duty_cycle_R);
    //ledcWrite(2, 0);
    ledcWrite(3, 0);
  Serial.print(", duty_cycle_R = ");
  Serial.println(duty_cycle_R);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Turn_left(){
  ledcWrite(0, 0);
  ledcWrite(1, 1800);
  ledcWrite(2, 1800);
  ledcWrite(3, 0);
  delay(125);
  ledcWrite(0, 3000);
  ledcWrite(1, 0);
  ledcWrite(2, 3000);
  ledcWrite(3, 0);
  delay(150);
  ledcWrite(0, 0);
  ledcWrite(1, 1800);
  ledcWrite(2, 1800);
  ledcWrite(3, 0);
  delay(375);
}

void Turn_right(){
  ledcWrite(0, 0);
  ledcWrite(1, 1800);
  ledcWrite(2, 1800);
  ledcWrite(3, 0);
  delay(125);
  ledcWrite(0, 0);
  ledcWrite(1, 3000);
  ledcWrite(2, 0);
  ledcWrite(3, 3000);
  delay(150);
  ledcWrite(0, 0);
  ledcWrite(1, 1800);
  ledcWrite(2, 1800);
  ledcWrite(3, 0);
  delay(375);
}

void setup() {
  neopixelWrite(38,0* 20,0 * 20, 0 );
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
unsigned long m_print = 0;
void loop() {
  if((millis()-t)>LOOP_TIME_MS){
    t=millis();
    getEn_ms(0);
    getEn_ms(1);

    move_IR(0);
  // Serial.print("point_IR_L = ");
  // Serial.print(point_IR[0]);

  // Serial.print("PID_IR_L = ");
  // Serial.print(output_PID_IR[0]);
    
    move_IR(4);
  // Serial.print(", point_IR_R = ");
  // Serial.println(point_IR[4]);

  // Serial.print(", PID_IR_R = ");
  // Serial.println(output_PID_IR[4]);

  // Serial.print(" L = ");
  // Serial.print(IRread(0));
  // Serial.print(" R = ");
  // Serial.println(IRread(4));

  // Serial.print("speed_L = ");
  // Serial.print(speed[0]);
  // Serial.print(" ,speed_R = ");
  // Serial.println(speed[1]);

  //point(10);

  // Lmotor(3500);
  // Rmotor(3500);
  Lmotor(3300);
  Rmotor(3300);
  }
  if((millis() - m_print) >= 1000){
  m_print = millis();
  
  // Serial.print(" L = ");
  // Serial.print(enc1.getCount());
  // Serial.print(" R = ");
  // Serial.println(enc2.getCount());

  
  // Serial.print("error_IR_L = ");
  // Serial.print(error_IR[0]);
  // Serial.print(", error_IR_R = ");
  // Serial.println(error_IR[4]);

  // Serial.print("PID_IR_L = ");
  // Serial.print(output_PID_IR_L);
  // Serial.print(", PID_IR_R = ");
  // Serial.println(output_PID_IR_R);

  }
} 
