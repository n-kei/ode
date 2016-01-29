#include <stdio.h>

 // PI制御ゲイン
#define GAIN_P 5.0 // 比例ｹﾞｲﾝ
#define GAIN_I 1.0 // 積分ｹﾞｲﾝ
// 運動モデル
#define K_POS 1.0 // バネ力
#define K_VEL 2.0 // ダンピング
#define MASS 1.0 // 質量
#define INT_TIME 0.1 // 時間ｽﾃｯﾌﾟ

//PI制御
double PIctrl(double dCommand, double dVal)
 {
 static double s_dErrIntg = 0;
 double dErr;
 double dRet;

 // 誤差
dErr = dCommand - dVal;

 // 誤差積分
s_dErrIntg += dErr * INT_TIME;

 // 制御入力
dRet = GAIN_P * dErr + GAIN_I * s_dErrIntg;

 return (dRet);
 }
 //ここまで

// 検証用プログラム
void main()
 {
 double dF, dFctrl;
 double dAcc, dVel, dPos, dTime;
 int i;

 // 初期値
dVel = 0; // 速度
dPos = 0; // 位置
dTime = 0; // 時刻

for (i = 0; i < 500; i++) {
 // コントローラ
dFctrl = PIctrl(10, dPos);

 // 外力
dF = dFctrl - (K_POS * dPos + K_VEL * dVel);

 // 加速度
dAcc = dF / MASS;

 // 速度
dVel += dAcc * INT_TIME;

 // 位置
dPos += dVel * INT_TIME;

 // 時刻
dTime += INT_TIME;

 printf("TIME = %f, POS = %f\n", dTime, dPos);
 }
 }