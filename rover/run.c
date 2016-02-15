#include <stdio.h>
#include <string.h>
#include <ode/ode.h>
#include "run.h"

#define _USE_MATH_DEFINES

#include <math.h>

//#define SERIAL_ENABLE

static clock_t GetCoordinateRate;
static clock_t MeasureGyroRate;
static clock_t GetCoordinateOrigin;
static clock_t MeasureGyroOrigin;

static float GetCoordinateNoise;
static float MeasureGyroNoise;

#ifdef SERIAL_ENABLE
void ReceiveRequestData(dBodyID *body, int h, char *request)
{
  char *RequestPhrase;
  char *RequestData[4];
  int i;

  RequestPhrase = strtok(request, ",");
  if(strcmp("MotorControl", RequestPhrase) == 0) {
    for(i = 0; i < 2; i++) {
      RequestData[i] = strtok(NULL, ",");
    }
    MotorControl(atoi(RequestData[0]), atoi(RequestData[1]));
  }

  else if(strcmp("GetCoordinate", RequestPhrase) == 0) {
    float x, y, z;
    char SendData[100];

    GetCoordinate(body, &x, &y, &z);
    sprintf(SendData, "%f,%f,%f,", x, y, z);
    fprintf(stderr, "SendData = %s\n", SendData);
    SerialSendData(h, SendData);
  }

  else if(strcmp("MeasureGyro", RequestPhrase) == 0) {
    float x, y, z;
    char SendData[100];

    MeasureGyro(body, &x, &y, &z);
    sprintf(SendData, "%f,%f,%f,", x, y, z);
    SerialSendData(h, SendData);
  }

  else if(strcmp("SetGoalPoint", RequestPhrase) == 0) {
    for(i = 0; i < 2; i++) {
      RequestData[i] = strtok(NULL, ",");
    }
    SetGoalPoint((float)atof(RequestData[0]), 
		 (float)atof(RequestData[1]));
  }

  else if(strcmp("SetGoalBox", RequestPhrase) == 0) {
    for(i = 0; i < 3; i++) {
      RequestData[i] = strtok(NULL, ",");
    }
    SetGoalBox((float)atof(RequestData[0]),
	       (float)atof(RequestData[1]),
	       (float)atof(RequestData[2]));
  }

  else if(strcmp("GetAngle", RequestPhrase) == 0) {
    float originX, originY, destX, destY;
    float angle;
    char SendData[100];

    for(i = 0; i < 4; i++) {
      RequestData[i] = strtok(NULL, ",");
    }
    originX = (float)atof(RequestData[0]);
    originY = (float)atof(RequestData[1]);
    destX = (float)atof(RequestData[2]);
    destY = (float)atof(RequestData[3]);

    angle = GetAngle(originX, originY, destX, destY);
    sprintf(SendData, "%f", angle);
    SerialSendData(h, SendData);
  }

  else if(strcmp("GetDistance", RequestPhrase) == 0) {
    float originX, originY, destX, destY;
    float angle;
    char SendData[100];

    for(i = 0; i < 4; i++) {
      RequestData[i] = strtok(NULL, ",");
    }
    originX = (float)atof(RequestData[0]);
    originY = (float)atof(RequestData[1]);
    destX = (float)atof(RequestData[2]);
    destY = (float)atof(RequestData[3]);

    angle = GetAngle(originX, originY, destX, destY);
    sprintf(SendData, "%f", angle);
    SerialSendData(h, SendData);

  }

}
#endif

void SetSamplingRate(int FuncID, clock_t rate)
{
  switch(FuncID) {
  case GETCOORDINATE:
    GetCoordinateRate = rate;
    GetCoordinateOrigin = clock();
    break;
  case MEASUREGYRO:
    MeasureGyroRate = rate;
    MeasureGyroOrigin = clock();
    break;
  default:
    break;
  }
}

void SetNoiseValue(int FuncID, float noise)
{
  switch(FuncID) {
  case GETCOORDINATE:
    GetCoordinateNoise = noise;
    break;
  case MEASUREGYRO:
    MeasureGyroNoise = noise;
    break;
  default:
    break;
  }
}

float GetNoiseValue(int FuncID)
{
  float NoiseValue;
  
  switch(FuncID) {
  case GETCOORDINATE:
    if(GetCoordinateNoise == 0)
      return(0);
    NoiseValue = ((float)(rand() % (2*(int)GetCoordinateNoise)) + (float)rand() / ((float)RAND_MAX + 1)) - GetCoordinateNoise;
    break;
    
  case MEASUREGYRO:
    if(MeasureGyroNoise == 0)
      return(0);
    NoiseValue = ((float)(rand() % (2*(int)MeasureGyroNoise)) + (float)rand() / ((float)RAND_MAX + 1)) - MeasureGyroNoise;
    break;
    
  default:
    NoiseValue = 0.0;
    break;
  }

  return(NoiseValue);
}

void MotorControl(int motorL, int motorR)
{
  int motorl, motorr;

  /*
    if(motorL > 255)
    motorl = 255;
    else
    motorl = motorL;
  
    if(motorR > 255)
    motorr = 255;
    else
    motorr = motorR;
  */
  speedL = (dReal)motorL / 255.0 * MAX_MOTOR_L;
  speedR = (dReal)motorR / 255.0 * MAX_MOTOR_R;
}

int GetCoordinate(dBodyID *body, float *x, float *y, float *z)
{
  const dReal *pos;

  if((clock() - GetCoordinateOrigin) >= GetCoordinateRate) {
    pos = dBodyGetPosition(body[0]);
    *x = (float)pos[0] + GetNoiseValue(GETCOORDINATE);
    *y = (float)pos[1] + GetNoiseValue(GETCOORDINATE);
    *z = (float)pos[2] + GetNoiseValue(GETCOORDINATE);
    GetCoordinateOrigin = clock();
    return(AVAILABLE_DATA);
  } else
    return(DISABLE_DATA);
  
}

int GetCoordinateAve(dBodyID *body, float *x, float *y, float *z)
{
  const dReal *pos;
  static float xave = 0, yave = 0, zave = 0;
  static unsigned int cnt = 0;

  pos = dBodyGetPosition(body[0]);
  xave += (float)pos[0] + GetNoiseValue(GETCOORDINATE);
  yave += (float)pos[1] + GetNoiseValue(GETCOORDINATE);
  zave += (float)pos[2] + GetNoiseValue(GETCOORDINATE);
  cnt++;
  
  if((clock() - GetCoordinateOrigin) >= GetCoordinateRate) {
    *x = xave / cnt;
    *y = yave / cnt;
    *z = zave / cnt;
    xave = yave = zave = 0.0;
    cnt = 0;
    GetCoordinateOrigin = clock();
    return(AVAILABLE_DATA);
  } else
    return(DISABLE_DATA);
  
}

int MeasureGyro(dBodyID *body, float *x, float *y, float *z)
{
  const dReal *gyro;

  if((clock() - MeasureGyroOrigin) >= MeasureGyroRate) {
    gyro = dBodyGetAngularVel(body[0]);
    *x = (float)gyro[0] + GetNoiseValue(MEASUREGYRO);
    *y = (float)gyro[1] + GetNoiseValue(MEASUREGYRO);
    *z = (float)gyro[2] + GetNoiseValue(MEASUREGYRO);
    MeasureGyroOrigin = clock();  
    return(AVAILABLE_DATA);
    
  } else
    return(DISABLE_DATA);

}

void SetGoalPoint(float x, float y)
{
  goalX = x;
  goalY = y;
}

void SetStartPoint(float x, float y, float z)
{

}

void SetGoalBox(float width, float length, float height)
{
  goalWidth = width;
  goalLength = length;
  goalHeight = height;
}

float GetCrossProduct(float originX, float originY,
		      float destX, float destY)
{
  float cross;
  
  cross = originX * destY - destX * originY;
  
  return(cross);
}

float GetInnerProduct(float originX, float originY,
		      float destX, float destY)
{
  float inner;
  
  inner = originX * destX + originY * destY;
  
  return(inner);
}

float GetDistance(float originX, float originY,
		  float destX, float destY)
{
  float distance;
  
  distance = (float)sqrt(pow((long double)(destX - originX), 2) + pow((long double)(destY - originY), 2));

  return(distance);
}

float GetAngle(float originX, float originY, 
	       float destX, float destY)
{
  static float angle = 0.0;
  float inner, cross;
  float originDis, destDis;
  
  originDis = GetDistance(0, 0, originX, originY);
  destDis = GetDistance(0, 0, destX, destY);
  if(originDis != 0 && destDis != 0) {
    inner = GetInnerProduct(originX, originY, destX, destY);
    cross = GetCrossProduct(originX, originY, destX, destY);
    if(inner / (originDis * destDis) < 0.99999999999999
       || inner /(originDis * destDis) > 1.000000000000001)
      angle = acosf(inner / (originDis*destDis));

    if(cross < 0)
      angle = -angle;
    return(angle);
  }
  
  return(0);
}

float GetAngle4Vector(VECTOR vec1, VECTOR vec2)
{
  static float angle = 0.0;
  float inner, cross;
  float originDis, destDis;
  
  originDis = GetDistance(vec1.x1, vec1.y1, vec1.x2, vec1.y2);
  destDis = GetDistance(vec2.x1, vec2.y1, vec2.x2, vec2.y2);
  if(originDis != 0 && destDis != 0) {
    inner = GetInnerProduct(vec1.x2 - vec1.x1, vec1.y2 - vec1.y1,
			    vec2.x2 - vec2.x1, vec2.y2 - vec2.y1);
    cross = GetCrossProduct(vec1.x2 - vec1.x1, vec1.y2 - vec1.y1,
			    vec2.x2 - vec2.x1, vec2.y2 - vec2.y1);
    if(inner / (originDis * destDis) < 0.99999999999999
       || inner /(originDis * destDis) > 1.000000000000001)
      angle = (float)acos(inner / (originDis*destDis));

    if(cross < 0)
      angle = -angle;
    return(angle);
  }
  
  return(0);

}
