#include <stdio.h>
#include <time.h>
#include "run.h"
#include "control.h"

#define GOAL_X 10.0
#define GOAL_Y 10.0

 // PI制御ゲイン
#define GAIN_P 100.0 // 比例ｹﾞｲﾝ
#define GAIN_I 0.00000005 // 積分ｹﾞｲﾝ
#define GAIN_D 0.0 //微分ゲイン

#define Ka 0.01

float constrain(float val, float min, float max)
{
  if(val < min)
    return(min);
  else if(val > max)
    return(max);
  else
    return(val);
}

float PIDctrl(float dCommand, float dVal, float dt)
{
  static float s_dErrIntg = 0;
  static float s_iErrDet = 0;
  static float dRet = 0;
  float dErr, iErr;
  
  // 誤差
  dErr = dCommand - dVal;
  // 誤差積分
  s_dErrIntg += dErr * dt;
  // 誤差微分
  iErr = dErr - s_iErrDet;
  //if(iErr < 0)
  //  iErr = -iErr;  
  s_iErrDet = dErr;
  // 制御入力
  dRet = GAIN_P * dErr + GAIN_I * s_dErrIntg + GAIN_D * iErr;

  //fprintf(stderr, "de:%f\tie:%f\n", dErr, iErr);
  return (dRet);
}

void SteerControl(float Command_rad, float Current_rad, float dt)
{
  float ControlValue;

  ControlValue = PIDctrl(Command_rad, Current_rad, dt);
  if(ControlValue < 0) {
    ControlValue = -ControlValue;
    ControlValue = constrain(ControlValue, 1, 255);
    //fprintf(stderr, "cr:%f\t", ControlValue);
    MotorControl(255, 255 - ControlValue); 
    //MotorControl(255 - ControlValue, 255);
  }
  else{
    ControlValue = constrain(ControlValue, 1, 255);
    //fprintf(stderr, "cl:%f\t", ControlValue);
    MotorControl(255 - ControlValue, 255);
    //MotorControl(255, 255 - ControlValue); 
  }
}

float getDt(void)
{
  static clock_t origin = 0;
  clock_t dt;

  dt = clock() - origin;
  origin = clock();
  
  return(dt);
}

FILE *fp;
clock_t StartTime;
float CurrentAngle;

void setup(void)
{
  SetGoalPoint(GOAL_X, GOAL_Y);
  if((fp = fopen("control1.dat", "w")) == NULL) {
    perror("control2.dat");
    exit(1);
  }
  fprintf(fp, "#CurrentValue,#time\n");
  SetSamplingRate(GETCOORDINATE, 100000);
  
  StartTime = clock();
  CurrentAngle = GetAngle(1, 0,GOAL_X, GOAL_Y);
} 

void loop(void)
{
  float x, y, z;
  float distance;
  float dt;
  int Sflag = DISABLE_DATA;

  
  dt = getDt();
  MeasureGyro(body, &x, &y, &z);
  Sflag = GetCoordinate(body, &x, &y, &z);
  
  if(Sflag) {
    distance = GetDistance(x, y, GOAL_X, GOAL_Y);
    CurrentAngle = GetAngle(x, y, GOAL_X, GOAL_Y);
    distance = distance * sin(CurrentAngle);
  }
  //SteerControl(angle+ distance * 0.1, z*dt/1000, dt);
  //fprintf(stderr, "angle=%f\tditance=%f\n", angle*RAD2DEG, distance);
  //fprintf(fp, "%f,%ld\n", angle*RAD2DEG, clock() - StartTime);
}
