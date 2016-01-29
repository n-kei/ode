#ifndef ROVER_H_INCLUDED
#define ROVER_H_INCLUDED

#include <math.h>
#include <ode/ode.h>

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0

#define MAX_MOTOR_R 3.0
#define MAX_MOTOR_L 3.0

#define GOAL_X 7.0 //[m]
#define GOAL_Y 7.0 //[m]

#define MAX_GOAL_DISTANCE 185 //[m] 種子島のフィールドにおいて走行しなければならない走行距離の最悪値

extern dReal speedR, speedL, steer;	// user commands
extern dReal goalX, goalY, goalZ, goalWidth, goalLength, goalHeight;

void MotorControl(int motorL, int motorR);
void GetCoordinate(dBodyID *body, float *x, float *y, float *z);
void MeasureGyro(dBodyID *body, float *x, float *y, float *z);
void SetGoalPoint(float x, float y);
void SetGoalBox(float width, float length, float height);
float GetCrossProduct(float originX, float originY,
	                  float destX, float destY);
float GetInnerProduct(float originX, float originY,
		              float destX, float destY);
float GetAngle(float originX, float originY,
	                  float destX, float destY);
float GetDistance(float originX, float originY,
			      float destX, float destY);
#endif
