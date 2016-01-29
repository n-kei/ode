#include <stdio.h>
#include <string.h>
#include <ode/ode.h>
#include "run.h"

#define _USE_MATH_DEFINES

#include <math.h>

//#define SERIAL_ENABLE

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

void GetCoordinate(dBodyID *body, float *x, float *y, float *z)
{
  const dReal *pos;
  
  pos = dBodyGetPosition(body[0]);
  *x = (float)pos[0];
  *y = (float)pos[1];
  *z = (float)pos[2];
}

void MeasureGyro(dBodyID *body, float *x, float *y, float *z)
{
  const dReal *gyro;

  gyro = dBodyGetAngularVel(body[0]);
  *x = (float)gyro[0];
  *y = (float)gyro[1];
  *z = (float)gyro[2];
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
    if(inner / (originDis * destDis) < 0.9999999999
       || inner /(originDis * destDis) > 1.000000001)
      angle = acosf(inner / (originDis*destDis));

    if(cross < 0)
      angle = -angle;
    return(angle);
  }
  
  return(0);
}
