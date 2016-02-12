#ifndef ROVER_H_INCLUDED
#define ROVER_H_INCLUDED

#include <math.h>
#include <ode/ode.h>
#include <time.h>

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0

#define MAX_MOTOR_R 2.5
#define MAX_MOTOR_L 2.5

#define GETCOORDINATE 0
#define MEASUREGYRO 1

#define DISABLE_DATA 0
#define AVAILABLE_DATA 1

#define MAX_GOAL_DISTANCE 185 //[m] 種子島のフィールドにおいて走行しなければならない走行距離の最悪値

extern dReal speedR, speedL, steer;	// user commands
extern dReal goalX, goalY, goalZ, goalWidth, goalLength, goalHeight;
extern dBodyID body[4];

typedef struct {
  float x1;
  float y1;
  float x2;
  float y2;
} VECTOR;

void ReceiveRequestData(dBodyID *body, int h, char *request);
void SetSamplingRate(int FuncID, clock_t rate);
void SetNoiseValue(int FuncID, float noise);
float GetNoiseValue(int FuncID);
void MotorControl(int motorL, int motorR);
int GetCoordinate(dBodyID *body, float *x, float *y, float *z);
int MeasureGyro(dBodyID *body, float *x, float *y, float *z);
void SetGoalPoint(float x, float y);
void SetStartPoint(float x, float y, float z);
void SetGoalBox(float width, float length, float height);
float GetCrossProduct(float originX, float originY,
	                  float destX, float destY);
float GetInnerProduct(float originX, float originY,
		              float destX, float destY);
float GetAngle(float originX, float originY,
	                  float destX, float destY);
float GetAngle4Vector(VECTOR vec1, VECTOR vec2);
float GetDistance(float originX, float originY,
			      float destX, float destY);
#endif
