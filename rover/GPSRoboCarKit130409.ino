// *****************************************************************************
//                        GPSロボットカーキット　        
//                                                      　　　葉山清輝（熊本高専）
// 
//  2011/11/28,  Ver. 1.0: 作成
//  2012/09/15,  サーボをゴム引きじゃなくて、針金操作にした。
//  　　　　　　 サーボのオフセットをEPROMに保存するように修正
//  2012/09/26   各種パラメータをスイッチで設定できるように改良
//  2013/4/5     モータを変えて動き出したのでパラメータ修正など
//  2013/4/6      GPS走行モードの追加（方位がとれないときにニュートラルに戻す） 
// *****************************************************************************

#include <EEPROM.h>
#include <Servo.h> 
#include <math.h>                       // atn，pow関数用

// 入力スイッチ
#define SET    7                        // セットスイッチ，デジタル入力
#define START  8                        // スタートスイッチ，デジタル入力 
// LED
#define LED0  10                        // 表示用LED（PB5）
#define LED1  11                        // 表示用LED（PB3）
#define LED2  12                        // 表示用LED（PB4）
#define LED3  13                        // 表示用LED（PB5）
// モータ制御用端子の定義
#define IN1   5                         // モータ制御入力１ 
#define IN2   6                         // モータ制御入力２
// 各種定数または初期値の設定
#define DEF_WR      2                   // ウェイポイントの半径
#define DEF_KS      0.1                 // ステアリング切れ角比例係数
#define DEF_STRMIN     80               // ステアリング左最大切れ角
#define DEF_STRMAX     120              // ステアリング右最大切れ角
#define DEF_POSN        100             // ステアリング停止時の中立値
#define DEF_SPD        150              // 走行時のモータの値
#define PI/2  1.5708                    // 3.14159/2
#define PI/180  0.01745                 // 3.14159/180

// ウェイポイント設定(基準点からのN,E方向への座標)、最高16ポイント
double wy[16];  
double wx[16];
int  wp=0;                              // wp:0-15まで,ウェイポイントの数 -1
int  np=0;                              // 目標ポイントナンバー

// 走行処理関係変数
double gy, gx;                          // 北緯，東経を原点からの距離（メートル）に数値化
double ry, rx, rv, rd;                  // 北緯（正規化），東経（正規化），速度（ノット），方向（単位：度）の数値化
double dy, dx, dr, dd;                  // ウェイポイントに対する北緯，東経の差分(m)，距離の差分，方位の差分

double wr;                              // ウェイポイントの半径
double ks;                              // ステアリング切れ角比例係数

int posn;                               // ニュートラルの位置 
int posSteer;                           // ステアリングの位置
float strmin;                           // ステアリングの最小値
float strmax;                           // ステアリングの最大値
int spd;                                // スピード

int pmode=0;                            // プログラムモード(メインメニューで分岐)

// GPS受信用変数，ポインタ
char str[100];                          // GPSほか読み取りのための文字列バッファ
char *latitude, *longitude, *knot, *direct;   
double latit, longit, kn;               // 緯度，経度，ノット保存用の変数
double latitB, longitB, rdB;            // 差分で方位計算する時に前の座標保持用, 方位保管用
double dy2, dx2, rd2;                   // 一つ前の値との北緯，東経の差分(m)，差分で求めた方位

Servo strServo; 

//-------------------------------------------------------------------------- 
// Double データのEEPROMへの読み書き	
//-------------------------------------------------------------------------- 
void EEPROM_writeDouble(int ee, double value)
{
    //byte* p = (byte*)(void*)&value;
    const byte* p = (const byte*)(const void*)&value;
    for (int i = 0; i < sizeof(value); i++)
      EEPROM.write(ee++, *p++);
}

double EEPROM_readDouble(int ee)
{
    double value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
      *p++ = EEPROM.read(ee++);
    return value;
} 

//-------------------------------------------------------------------------- 
// EEPROMにウェイポイントの保管と読み出し
// アドレス0-127にGPS座標（doubleで4バイトx2(wy,wx)x16）を記録
// アドレス128にウェイポイント数を記録
//-------------------------------------------------------------------------- 
void writePoint(){
  EEPROM.write(128,wp);
  for(int i=0;i<16;i++){
    EEPROM_writeDouble((i*8), wy[i]);    // ４バイトずつｗｙ，ｗｘで並べて書く
    EEPROM_writeDouble((i*8)+4, wx[i]);  
  }
}

void readPoint(){                      
  wp=EEPROM.read(128);
  for(int i=0;i<16;i++){
    wy[i]=EEPROM_readDouble((i*8));
    wx[i]=EEPROM_readDouble((i*8)+4);  
  }  
}

//-------------------------------------------------------------------------- 
// EEPROMに走行パラメータを読み書き
//-------------------------------------------------------------------------- 
void writeParam(){
  EEPROM.write(129,wr*10);            // 実数はEEPROMに記録できないので１０倍して記録
  EEPROM.write(130,ks*100);           // 実数はEEPROMに記録できないので１００倍して記録
  EEPROM.write(131,strmin);
  EEPROM.write(132,strmax);
  EEPROM.write(133,posn);
  EEPROM.write(134,spd);
}

void readParam(){
  wr=(double)EEPROM.read(129)/10;     // １０倍して記録してあるので１／１０
  ks=(double)EEPROM.read(130)/100;    // １００倍して記録してあるので１／１００
  strmin=EEPROM.read(131);
  strmax=EEPROM.read(132);
  posn=EEPROM.read(133);
  spd=EEPROM.read(134);
}

//-------------------------------------------------------------------------- 
//  ウェイポイントの設定   
//  (GPSが受信できてない時はポイント追加できない)
//-------------------------------------------------------------------------- 
void setPoint(){
  
  dispLED(wp); // ウェイポイント番号をLEDに2進表示
  while (digitalRead(START)==HIGH){      // STARTスイッチが押されたら抜ける
    
    // セットスイッチが押されたらウェイポイントを記録   
    if (digitalRead(SET)==LOW) {      
      delay(10);                      
      while(digitalRead(SET)==LOW);      // チャタリング防止
      delay(10);     
      
      // GPRMCを受信できて値が正常であるまで待つ 
      while(recvGPS()==0 || (latit==0) || (longit==0));  
      
      // ウェイポイント追加
      wy[wp] = latit; 
      wx[wp] = longit; 
      writePoint();            // EEPROMにウェイポイントの書き出し         
      wp++; if (wp>15) wp=15;  // 次のウェイポイント入力状態にするウェイポイントの追加は１５まで、０からで計１６個  
      dispLED(wp);  
    }
  }
}
  
//-------------------------------------------------------------------------- 
//	ウェイポイントの消去
//-------------------------------------------------------------------------- 
void clearPoint(){
  wp=0;
  for(int i=0;i<16;i++){ wy[i]=0; wx[i]=0; }
  writePoint();                // EEPROMにウェイポイントの書き出し
}

//-------------------------------------------------------------------------- 
//  LED表示	
//  LED2(digital13:PB5),LED1(digital12:PB4),LED0(digital11:PB3)	を２進数表示
//  dispLEDはループの中で頻繁に呼ばれると、サーボが誤動作するので注意！！
//-------------------------------------------------------------------------- 
void dispLED(byte n)
{
  PORTB &= B11000011;                     // LEDを一旦消す
  PORTB |= (n<<2);                        // LED表示
}

// LEDを指定した表示で3回点滅させる
void blink(int n){
  for (int i=0;i<3;i++){ dispLED(n); delay(100); dispLED(0); delay(100); }
}

//--------------------------------------------------------------------------
//  シリアルポートで文字列受信
//--------------------------------------------------------------------------
void recvStr(char *buf)
{
  int i = 0;
  char c;
  while (1) {
    if (Serial.available()) {
      c = Serial.read();
      buf[i] = c;
      if (c == '\n') break; 
      i++;
    }
  }
  buf[i] = '\0';  // \0: end of string
}

//--------------------------------------------------------------------------
//  GPS受信		                                                  
//--------------------------------------------------------------------------
int recvGPS(){
  if (Serial.available()) {  
    recvStr(str);   
    if(strcmp(strtok(str,","),"$GPRMC")==0){   //if RMC line
      strtok(NULL,",");
      strtok(NULL,",");
      latitude=strtok(NULL,","); //get latitude
      strtok(NULL,",");
      longitude=strtok(NULL,","); //get longtude
      strtok(NULL,",");          // E読み飛ばし
      knot=strtok(NULL,",");     // 速度 get
      direct=strtok(NULL,",");   // 進行方向 get 
      
      gps_val();        //	GPS信号の数値返還             
      gps_cal();       // 現在地とウェイポイントとの関係を計算  
      return(1);
    }
  }
  return(0);
}

//-------------------------------------------------------------------------- 
//  GPS信号の数値返還       
//-------------------------------------------------------------------------- 
void gps_val(){

      latitB=latit; longitB=longit;  // 一つ前の値を保管
      latit=atof(latitude+2);        // 文字列を実数に変換．有効数字が足りないので度単位を省略
      longit=atof(longitude+3);      // 文字列を実数に変換．有効数字が足りないので度単位を省略

      kn=atof(knot);
      rv=kn*0.51;                    // ノット単位をメートル単位に変換
      rdB=rd;                        // 一つ前の値を保管
      rd=atof(direct);               // 方位を数値変換 
}

//-------------------------------------------------------------------------- 
//  現在地とウェイポイントとの相対関係を計算      
//-------------------------------------------------------------------------- 
void gps_cal(){
      // 緯度経度の数値変換，目標地点との差分を計算
      dy=(wy[np] - latit ) *1860; 
      dx=(wx[np] - longit)*1560;
      
      // 目標地点までのと距離を計算
      dr=sqrt(pow(dy,2)+pow(dx,2));
      
      // 目標地点への方向を計算 
      dd = atan(dx / dy);                 // 北に対する角度を求める(±π/2範囲)
      dd=dd*57;                           // ラジアン->度に変換 dd*(180/pai)
      // 0-360度に変換
      if (dx > 0 && dy < 0)     dd = 180 + dd;
      else if(dx < 0 && dy < 0) dd = 180 + dd;
      else if(dx < 0 && dy > 0) dd = 360 + dd;
      else;
 
      // GPSが正しい方位を出力しない場合は、前の座標との差分で計算
      dy2=(latit -latitB) *1860; 
      dx2=(longit-longitB)*1560;
      
      if (dy2==0 || dx2==0){                  // 緯度または経度のどちらかの変化が０の時
        if (dy2==0){ if (dx2<0) rd2= 270; else rd2=90; } // rd2= 0, 90, 180，270 のいずれかを直接与える．
        if (dx2==0){ if (dy2<0) rd2=-180; else rd2=0; } // dy2,dx2のどちらも0の時はrd2=0
      } else {
        // 目標地点への方向を計算
        rd2 = atan(dx2 / dy2);                 // 北に対する角度を求める(±π/2範囲)
        rd2=rd2*57;                           // ラジアン->度に変換 dd*(180/pai)
        // 0-360度に変換
        if (dx2 > 0 && dy2 < 0)     rd2 = 180 + rd2;
        else if(dx2 < 0 && dy2 < 0) rd2 = 180 + rd2;
        else if(dx2 < 0 && dy2 > 0) rd2 = 360 + rd2;
        else;
      }
      
      // 移動距離が小さくて使用するGPSが正しい速度と方向を出さない時のみ
      // 座標差から計算した値と置き換える（速度=0,方位前の値のまま，GPSの仕様により異なる）
      if (kn==0 && dx2!=0 && dy2!=0){  
        rd=rd2; 
      } 
      
      // 方位偏差の計算し，現在の進行方向から±180度の範囲に直す
      dd=dd-rd;
      if (dd>180) dd=dd-360;
      else if (dd<-180) dd=dd+360;  
}

//-------------------------------------------------------------------------- 
//  モータスタート
//-------------------------------------------------------------------------- 
void motorStart(){
  digitalWrite(IN1,LOW);
  analogWrite(IN2, spd); 
  delay(1000);
}

//-------------------------------------------------------------------------- 
//  モータストップ
//-------------------------------------------------------------------------- 
void motorStop(){
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,LOW);
}

//-------------------------------------------------------------------------- 
//  ステアリングニュートラル
//-------------------------------------------------------------------------- 
void steerNeutral(){
  strServo.write(posn); 
}

//--------------------------------------------------------------------------
//  GPSによる走行
//  runMode=0:GPSにより方位が取れなくても偏差で計算
//  runMode=1:方位がとれない場合はニュートラルに戻す		                                                  
//--------------------------------------------------------------------------
void runGPS(int runMode){
  
  np=0;
  motorStart();
  
  while (digitalRead(START)==HIGH){  // STARTスイッチが押されたら抜ける
   
    while (recvGPS()==0);            // GPS受信待ち  
    if (dr>15) dispLED(15); else dispLED((int)dr);  // 目標値までの距離をLEDに表示
    
    // 角度差に応じて方向にステアリングを切る．
    posSteer=posn + dd*ks;
    posSteer=constrain(posSteer, strmin,strmax);  // 切れ幅を制限する
    if (runMode==1 && kn==0) posSteer=posn; // GPSで方位が得られなかった時（速度＝０）ニュートラルに戻す
    strServo.write(posSteer); 
    
    // ウェイポイントとの距離を求め，ポイント更新または走行終了判断
    if (dr < wr){
      motorStop();
      steerNeutral();
      np++; if (np>wp) break;  // 次のポイントにすすめる。全ウェイポイント走ったら終了
      for (int i=0;i<5;i++){   // 次のポイント数をLEDに点滅表示しながら５秒待つ
        dispLED(np);
        delay(500);
        dispLED(0);
        delay(500);
      }
      motorStart();
    }
  }
}

//-------------------------------------------------------------------------- 
//  セットアップ以外の初期値設定
//-------------------------------------------------------------------------- 
void initPara(){
  steerNeutral();
  motorStop();
}

//-------------------------------------------------------------------------- 
//  セットアップ
//-------------------------------------------------------------------------- 
void setup() 
{ 
  pinMode(IN1, OUTPUT);    // モータ制御入力１を出力に定義
  pinMode(IN2, OUTPUT);    // モータ制御入力２を出力に定義
  digitalWrite(IN1,LOW); // モータ停止 
  digitalWrite(IN2,LOW); // モータ停止 
  
  pinMode(SET,   INPUT);    // セットスイッチ
  pinMode(START, INPUT);    // スタートスイッチ
  
  pinMode(LED0, OUTPUT);    // 状態モニタ用LED
  pinMode(LED1, OUTPUT);    // 状態モニタ用LED
  pinMode(LED2, OUTPUT);    // 状態モニタ用LED
  pinMode(LED3, OUTPUT);    // 状態モニタ用LED 
  
  // サーボ初期化
  strServo.attach(9); 
  strServo.write(posSteer); 
  
  // シリアル通信の初期化
  Serial.begin(9600);    // ハードウェアシリアルの速度を設定（できるだけ早く）

  readPoint(); // EEPROMからウェイポイントの読み出し
  
  // EEPROMからパラメータを読出し
  // セットスイッチを押しながらリセットをかけた場合は，パラメータをデフォルト値に戻す
  if (digitalRead(SET)==HIGH){    
    readParam();
  } else {                        // パラメータをデフォルト値に戻す
    wr=DEF_WR;          
    ks=DEF_KS;
    strmin=DEF_STRMIN; 
    strmax=DEF_STRMAX;
    posn=DEF_POSN;
    spd=DEF_SPD;
    writeParam();
  }
}  

//--------------------------------------------------------------------------
//  メインループ 		                                                  
//--------------------------------------------------------------------------
void loop()
{
  initPara();
  blink(15);
  dispLED(pmode);     // プログラムモードをLEDに2進表示  
  
  while (digitalRead(START)==HIGH){   // スタートスイッチが押されたら指定プログラムを実行

    if (digitalRead(SET)==LOW) {      // セットスイッチが押されたらプログラムモードを進める
      delay(10);                      // チャタリング防止
      while(digitalRead(SET)==LOW);
      delay(10);                      // チャタリング防止
      pmode++; if (pmode>15) pmode=0;  // プログラムモードを一つ増やす
      dispLED(pmode);     // プログラムモードをLEDに2進表示 
    }
  }  
  // スタートスイッチが押されたら処理プログラムへ分岐
  dispLED(0);
  delay(500);                         // 0.5秒待つ

  switch(pmode){                      // 選択されたプログラムモードのプログラムを実行
    case  0:  testServo();  break;    // サーボの動作テスト
    case  1:  testMotor();  break;    // モーターの動作テスト
    case  2:  testGPS();    break;    // GPSのテスト   
    case  3:  setPoint();   break;    // ウェイポイントのセット   
    case  4:  clearPoint(); break;    // ウェイポイントのクリア  
    case  5:  runGPS(0);    break;    // GPS走行(方位がとれなくても偏差で計算)  
    case  6:  runGPS(1);    break;    // GPS走行(方位がとれなかった時はニュートラルに戻す)  
    case  7:  setNeutral(); break;    // ステアリングニュートラル位置の設定
    case  8:  setStrmin();  break;    // ステアリングの最小値の設定
    case  9:  setStrmax();  break;    // ステアリングの最大値の設定
    case  10: setWr();      break;    // ウェイポイントの半径設定
    case  11: setKs();      break;    // 舵角計算の比例値設定
    case  12: setSpd();     break;    // 走行スピードの設定
    case  13: wpOut();      break;    // ウェイポイントのシリアル出力
    case  14: paramOut();   break;    // パラメータのシリアル出力
  }
  delay(500);                         // 0.5秒待つ
}


//--------------------------------------------------------------------------
//  テスト，パラメータ設定	                                                  
//--------------------------------------------------------------------------

// サーボのテスト
void testServo() {
  strServo.write(strmin);    // 最初左に曲がる
  delay(1000);  
  strServo.write(strmax);    // 次に右に曲がる
  delay(1000);
  strServo.write(posn);      // ニュートラル
  delay(1000);
}

// モータテスト、実行スイッチで正転、セットスイッチで逆転、リセットを押すまでモータ回り続ける
void testMotor(){
  while(1){
     if      (digitalRead(SET)==LOW)   { digitalWrite(IN2,LOW); analogWrite(IN1, spd); } 
     else if (digitalRead(START)==LOW) { digitalWrite(IN1,LOW); analogWrite(IN2, spd); } 
     // else {digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);}  
  }
}

// GPS受信テスト，補足状況をLED点灯の数で示す
void testGPS(){
  double latitp, longitp;
  while (digitalRead(START)==HIGH){  // STARTスイッチが押されたら抜ける
    if (recvGPS()==1){
      if (latit==0) dispLED(1);                                            // GPS補足なし
      else if((latitp==latit) && (longitp==longit)) dispLED(7);            // GPS補足あり、１秒前との値が同じ
      else dispLED(3);                                                     // GPS補足あり、１秒前との差が大きい時
      delay(100);
      dispLED(0);    // 0.1秒点灯させて消す
      latitp=latit; longitp=longit;                                        // 比較のため座標を記録
    }
  }
}

// ニュートラルのセット、スタートでプラス、セットでマイナス、リセットで戻る
void setNeutral(){
  while(1){
    if (digitalRead(START)==LOW) if (posn<strmax) posn++; 
    if (digitalRead(SET)==LOW)   if (posn>strmin) posn--; 
    strServo.write(posn);      // ニュートラル
    writeParam();
    delay(100);
  }
}

// ステアリング最小値のセット、スタートでプラス、セットでマイナス、リセットで戻る
void setStrmin(){
  while(1){
    if (digitalRead(START)==LOW) if (strmin<90) strmin++;   // 可変範囲は45-90度まで
    if (digitalRead(SET)==LOW)   if (strmin>45) strmin--; 
    strServo.write(strmin);      // ステアリング最小値に動かす
    writeParam();
    delay(100);
  }
}

// ステアリング最大値のセット、スタートでプラス、セットでマイナス、リセットで戻る
void setStrmax(){
  while(1){
    if (digitalRead(START)==LOW) if (strmax<135) strmax++;   // 可変範囲は90-135度まで
    if (digitalRead(SET)==LOW)   if (strmax>90 ) strmax--; 
    strServo.write(strmax);      // ステアリング最大値に動かす
    writeParam();
    delay(100);
  }
}

// ウェイポイントの半径をセット、スタートでプラス、セットでマイナス、リセットで戻る
// ２進数で半径を表示
void setWr(){
  while(1){
    if (digitalRead(START)==LOW) wr+=1;  
    if (digitalRead(SET)==LOW)  {wr-=1; if (wr<1) wr=1; } 
    dispLED(wr);
    writeParam();
    delay(200);
  }
}

// ステアリング制御の比例係数をセット、スタートでプラス、セットでマイナス、リセットで戻る
// 比例係数の50倍を２進数で表示
void setKs(){
  while(1){
    if (digitalRead(START)==LOW){ks+=0.02; if (ks>0.2) ks=0.2;}  
    if (digitalRead(SET)==LOW)  {ks-=0.01; if (ks<0.02) wr=0.02;} 
    dispLED(ks*16);
    writeParam();
    delay(200);
  }
}

// 直進スピードをセット、スタートでプラス、セットでマイナス、リセットで戻る
// スピードの1/16を2進数で表示
void setSpd(){
  while(1){
    digitalWrite(IN1,LOW); analogWrite(IN2, spd);  // モータを回転させる
    if (digitalRead(START)==LOW){spd+=8; if (spd>255) spd=255; }  
    if (digitalRead(SET)==LOW)  {spd-=8; if (spd<0) spd=0;} 
    dispLED(spd/16);
    writeParam();
    delay(200);
  }
}

// ウェイポイントの読み出し、表示。GPSのジャンパーを外して、シリアルポートに接続
void wpOut(){
  while(digitalRead(START)==HIGH){
    Serial.println(wp);
    for(int i=0;i<16;i++){
      Serial.print(wy[i],8); Serial.print(", "); Serial.println(wx[i],8);
    }
    delay(1000);
  }
}

// パラメータのシリアル出力
void paramOut(){
  Serial.print("wr    ="); Serial.println(wr); 
  Serial.print("ks    ="); Serial.println(ks); 
  Serial.print("strmin="); Serial.println(strmin); 
  Serial.print("strmax="); Serial.println(strmax); 
  Serial.print("posn  ="); Serial.println(posn); 
  Serial.print("spd   ="); Serial.println(spd); 
  Serial.println();
}
  
