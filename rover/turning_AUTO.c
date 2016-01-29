
#define msgn(x) Serial.println(x)
#define msg(x) Serial.print(x)
#define SW 12
#define MOTOR_R_FREE 4
#define MOTOR_R 5
#define MOTOR_L 6
#define MOTOR_L_FREE 7

#define Krc 0.2
#define KpR 2.5
#define KiR 0.05
#define KpT 2.5
#define KiT 0.05

volatile unsigned long TcntR,TcntL = {0};
volatile unsigned long cntR=0, cntL=0;
volatile unsigned long timR=0, timL=0;
void funcR(){encPulseR();}
void funcL(){encPulseL();}

float VR=0.0, VL=0.0;
float Rspeed=0.0, Lspeed=0.0;
float R_last=0.0, L_last=0.0;
float R_command = 0.0, T_command = 0.0; //max...96?

float v_Roll;
float v_Trans;
long angle;
long dist;

/**************************************/
/* 動作モード */
enum mode_t {
  MODE_STOP,      //停止
  MODE_STRAIGHT,  //まっすぐ進む
  MODE_TURN,      //左に曲がる
} mode;
/**************************************/

void setup() {
  // put your setup code here, to run once:
  //TCCR0A |= 0b10100000;
  TCCR2A = 0b00000011;
  TCCR2B = 0<<WGM02 | 0<<CS22 | 0<<CS21 | 1<<CS20;
  TIMSK2 = 1 << TOIE2;
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(13,OUTPUT);
  pinMode(MOTOR_R_FREE,OUTPUT);
  pinMode(MOTOR_L_FREE,OUTPUT);
  attachInterrupt(1, funcR, RISING);
  attachInterrupt(0, funcL, RISING);
  
  digitalWrite(MOTOR_R_FREE,LOW);
  digitalWrite(MOTOR_L_FREE,LOW);
  
  angle = 0;
  dist = 0;
  mode = MODE_STRAIGHT;
  
  Serial.begin(57600);
  
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly
  
  static unsigned long cntR_old;
  static unsigned long cntL_old;
  static unsigned int Rstop = 0;
  static unsigned int Lstop = 0;
  
  switch(mode){
    case MODE_STRAIGHT: R_command = 0.0; T_command = 20.0; 
                        if(dist > 6000)
                          {mode = MODE_TURN; dist=0;angle=0;} 
                        break;
    case MODE_TURN: R_command = 10.0; T_command = 10.0;
                        if(angle > 3440) //180deg?
                          {mode = MODE_STRAIGHT; dist=0;angle=0;} 
                        break;
  }
  
  /* Mesuring Verocity */
  if(100 < timR) VR = 1000000.0/(float)timR; 
  if(100 < timL) VL = 1000000.0/(float)timL;
  
  if(cntR_old == cntR) Rstop++; else Rstop = 0;
  if(cntL_old == cntL) Lstop++; else Lstop = 0;
  
  if(Rstop > 70) VR = 0.0;
  if(Lstop > 70) VL = 0.0;
  
  cntR_old = cntR; cntL_old = cntL;
  
  /* Smoothing Verocity Signal */
  
  Rspeed = Krc*VR + (1-Krc)*R_last; R_last = Rspeed;
  Lspeed = Krc*VL + (1-Krc)*L_last; L_last = Lspeed;
  
  //Rolling
  v_Roll = Rspeed - Lspeed;  
  angle += v_Roll;
  
  //translation
  v_Trans = Rspeed + Lspeed;
  dist += v_Trans;
  
  int Rout=0, Lout=0;
  
  //Control Rolling
  float R_error = R_command - v_Roll;
  static float R_Ierror = 0.0;
  float R_CV = KpR*R_error + KiR*R_Ierror;
  R_Ierror += R_error;
  Rout += R_CV; Lout -= R_CV;
  //Control Translation
  float T_error = T_command - v_Trans;
  static float T_Ierror = 0.0;
  float T_CV = KpT*T_error + KiT*T_Ierror;
  T_Ierror += T_error;
  Rout += T_CV; Lout += T_CV;
  
  //CV reflecting
  Rout = constrain(Rout, 0, 0xff);
  Lout = constrain(Lout, 0, 0xff);
  //OCR0A=Rout; OCR0B=Lout;
  analogWrite(MOTOR_R, Rout);
  analogWrite(MOTOR_L, Lout);
  /*
  msg(R_error);
  msg(",");
  msg(R_command);
  msg(",");
  msg(T_error);
  msg(",");
  msgn(T_command);
  */
  msgn(micros());
  intervalDelay_usec(2000);
}

//前回この関数が終了してから、period[ms]経過するまで待つ
void intervalDelay_usec(unsigned int period){
	static unsigned long last_time = 0;
	unsigned long now_time = micros();
	PORTB |= _BV(PB5);
	while( (now_time - last_time) < period ){ 
		now_time = micros(); 
	}
	PORTB &= ~_BV(PB5);
	last_time = now_time;
}

ISR(TIMER2_OVF_vect)//General Purpose Counter
{
  TcntR++; 
  TcntL++;
}

void encPulseR(void)
{
  timR = (TcntR << 8) + TCNT2; //Mesuring L_Pulse
  TcntR = 0;
  cntR++;                      //Counting R_distance 
}

void encPulseL(void)
{
  timL = (TcntL << 8) + TCNT2; //Mesuring L_Pulse
  TcntL = 0;
  cntL++;                      //Counting L_distance
}
