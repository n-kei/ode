#include <stdio.h>
#include <time.h>
#include "run.h"
#include "control.h"

#define GOAL_X 30.0
#define GOAL_Y 30.0

 // PI制御ゲイン
#define GAIN_P 6.0 // 比例ｹﾞｲﾝ
#define GAIN_I 0.05 // 積分ｹﾞｲﾝ
#define GAIN_D 0.8 //微分ゲイン

//#define SUCCESS1
#define SUCCESS2
//#define TEST

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
    ControlValue = constrain(ControlValue, 1, 125);
    //fprintf(stderr, "cr:%f\t", ControlValue);
    MotorControl(255, 255 - ControlValue); 
    //MotorControl(255 - ControlValue, 255);
  }
  else{
    ControlValue = constrain(ControlValue, 1, 125);
    //fprintf(stderr, "cl:%f\t", ControlValue);
    MotorControl(255 - ControlValue, 255);
    //MotorControl(255, 255 - ControlValue); 
  }
}

float GetTransitionAngle(float FormerAngle, float MinValue, float MaxValue)
{
  float mod;
  float AfterAngle;
  
  if(FormerAngle < MinValue) {
    mod = fmod(FormerAngle, fabs(MinValue));
    AfterAngle = MaxValue + mod;
  } else if(FormerAngle > MaxValue){
    mod = fmod(FormerAngle, fabs(MaxValue));
    AfterAngle = MinValue + mod;
  } else {
    AfterAngle = FormerAngle;
  }

  return(AfterAngle);
}

float getDt(void)
{
  static long lastTime=0;

  long nowTime = clock();
  float time = (float)(nowTime - lastTime);
  time = constrain(time,0, 20);  //timeは20[us]以上
  time /= 400;  //[usec] => [sec]
  lastTime = nowTime;

  return( time );
}

FILE *fp;
clock_t StartTime;

#ifdef SUCCESS1
void setup(void)
{
  SetGoalPoint(GOAL_X, GOAL_Y);
  if((fp = fopen("control1.dat", "w")) == NULL) {
    perror("control2.dat");
    exit(1);
  }
  fprintf(fp, "#CurrentValue,#time\n");

  //SetSamplingRate(GETCOORDINATE, 100000);
} 

void loop(void)
{
  float x, y, z;
  float distance, angle;
  float dt;
  int Sflag = DISABLE_DATA;
  
  dt = getDt();
  MeasureGyro(body, &x, &y, &z);
  GetCoordinate(body, &x, &y, &z);
  distance = GetDistance(0, 0, x, y);
  angle = GetAngle(x, y, GOAL_X, GOAL_Y);
  distance = distance * sin(angle);
  SteerControl(angle + distance * 0.15, z*dt/1000, dt);
  fprintf(stderr, "angle=%f\tditance=%f\n", angle*RAD2DEG, distance);
  fprintf(fp, "%f,%ld\n", angle*RAD2DEG, clock() - StartTime);
}
#endif

#ifdef SUCCESS2
float GoalAngle;
static VECTOR GoalVector, CurrentVector;
const float Ka = 1.0;
const float Kd = 0.0;

float originX = 0.0, originY = 0.0;

void setup(void)
{
  SetGoalPoint(GOAL_X, GOAL_Y);
  if((fp = fopen("control1.dat", "w")) == NULL) {
    perror("control2.dat");
    exit(1);
  }
  fprintf(fp, "#CurrentValue,#time\n");

  SetSamplingRate(GETCOORDINATE, 200000);
  SetNoiseValue(GETCOORDINATE, 1.0);
  GoalAngle = 45.0 * DEG2RAD;
  GoalVector.x2 = GOAL_X;
  GoalVector.y2 = GOAL_Y;
  CurrentVector.x2 = 0;
  CurrentVector.y2 = 0;
} 

void loop(void)
{
  static float x = 0, y = 0, z = 0;
  static float distance = 0;
  static float angle = 0;
  float dt;
  int Sflag = DISABLE_DATA;
  
  dt = getDt();
  MeasureGyro(body, &x, &y, &z);
  angle += z * dt;
  angle = GetTransitionAngle(angle, -180*DEG2RAD, 180*DEG2RAD);
  Sflag = GetCoordinate(body, &x, &y, &z);
  if(Sflag) {
    distance = GetDistance(0, 0, x, y);
    distance = distance * sin(GoalAngle);
    CurrentVector.x1 = CurrentVector.x2;
    CurrentVector.y1 = CurrentVector.y2;
    CurrentVector.x2 = GoalVector.x1 = x;
    CurrentVector.y2 = GoalVector.y1 = y;
    GoalAngle = GetAngle4Vector(CurrentVector, GoalVector);
    angle = 0;
    fprintf(stderr, "//////////GoalAngle/////=%f\n", GoalAngle*RAD2DEG);
  }
  SteerControl(GoalAngle, angle * Ka + distance * Kd, dt);
}
#endif

#ifdef TEST
static VECTOR current, goal;

void setup()
{
  SetGoalPoint(GOAL_X, GOAL_Y);
  current.x2 = 0;
  current.y2 = 0;
  goal.x2 = GOAL_X;
  goal.y2 = GOAL_Y;
  goal.x1 = 0.0;
  goal.y1 = 0.0;
}

void loop()
{
  float x, y, z, angle;

  GetCoordinate(body, &x, &y, &z);
  
  current.x1 = current.x2;
  current.y1 = current.y2;
  current.x2 = goal.x1 = x;
  current.y2 = goal.y1 = y;
  angle = GetAngle4Vector(current, goal);
  
  /*
  current.x1 = 0.0;
  current.y1 = 0.0;
  current.x2 = x;
  current.y2 = y;
  angle = GetAngle4Vector(current, goal); 
  */  
  fprintf(stderr, "angle=%f\n", angle*RAD2DEG);
  
}
#endif
