/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

buggy with suspension.
this also shows you how to use geom groups.

*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>
#include <time.h>
#include "texturepath.h"
#include "control.h"
#include "run.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// some constants

#define LENGTH 0.09	// chassis length
#define WIDTH 0.15	// chassis width
#define HEIGHT 0.08	// chassis height
#define RADIUS 0.07	// wheel radius
#define STARTZ 0.3	// starting height of chassis
#define CMASS 0.66		// chassis mass
#define WMASS 0.17	// wheel mass

#define POLE_MASS 0.25
#define POLE_LENGTH 0.1
#define POLE_RADIUS 0.025

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
dBodyID body[4];
static dJointID joint[3];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[3];
static dGeomID ground_box;

static dBodyID stbBody;
static dGeomID stbPole;
static dJointID stbJoint;
// things that the user controls

dReal speedR=0, speedL=0, steer=0;	// user commands
dReal goalX = 5.0, goalY = 5.0, goalZ = 0.0, goalWidth = 0.10, goalLength = 0.10, goalHeight = 1.0;

//#define RUN
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
}


// called when a key pressed

static void command (int cmd)
{
  switch (cmd) {
  case 'a': case 'A':
    speedR += 0.3;
	speedL += 0.3;
    break;
  case 'z': case 'Z':
    speedR -= 0.3;
	speedL -= 0.3;
    break;
  case ',':
    speedL += 0.3;
    break;
  case '.':
    speedR += 0.3;
    break;
  case 'l':
	speedR -= 0.3;
	break;
  case 'k':
	speedL -= 0.3;
	break;
  case ' ':
    speedR = 0;
    speedL = 0;
    break;
  }
}

// simulation loop

static void simLoop (int pause)
{
  int i;
  char buffer[100];
  char *RequestData = NULL;
  char ch;

  if (!pause) {
	float x, y, z;

	loop();
    // motor
	dJointSetHinge2Param (joint[0],dParamVel2,-speedL);
	dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

	dJointSetHinge2Param (joint[0],dParamVel,100);
	dJointSetHinge2Param (joint[0],dParamFMax,0.2);
	dJointSetHinge2Param (joint[0],dParamLoStop,0.0);
	dJointSetHinge2Param (joint[0],dParamHiStop,0.0);
	dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
	
	
	dJointSetHinge2Param (joint[1],dParamVel2,-speedR);
	dJointSetHinge2Param (joint[1],dParamFMax2,0.1);
	
	dJointSetHinge2Param (joint[1],dParamVel,0);
	dJointSetHinge2Param (joint[1],dParamFMax,0.2);
	dJointSetHinge2Param (joint[1],dParamLoStop,0.0);
	dJointSetHinge2Param (joint[1],dParamHiStop,0.0);
	dJointSetHinge2Param (joint[1],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
	
    dWorldStep (world,0.05);


    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }

  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,1,1);
  for (i=1; i<=2; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
				       dBodyGetRotation(body[i]),0.02f,RADIUS);
  
  dsDrawCapsule(dBodyGetPosition(stbBody),
				 dBodyGetRotation(stbBody), POLE_LENGTH, POLE_RADIUS);

  dVector3 ss;
  dsSetColor(1.0,0.0,0.0);  
  dGeomBoxGetLengths (ground_box,ss);
  dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);
  
  /*
  printf ("%.10f %.10f %.10f %.10f\n",
	  dJointGetHingeAngle (joint[1]),
	  dJointGetHingeAngle (joint[2]),
	  dJointGetHingeAngleRate (joint[1]),
	  dJointGetHingeAngleRate (joint[2]));
  */
}

int main (int argc, char **argv)
{
  int i;
  dMass m;
  // setup pointers to drawstuff callback functions
  dsFunctions fn;

  setup();

  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world, 0, 0, -0.5);
  ground = dCreatePlane (space,0,0,1,0);

  // chassis body
  body[0] = dBodyCreate (world);
  dBodySetPosition (body[0],0,0,STARTZ);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[0],&m);
  box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);

  
  // pole
  stbBody =  dBodyCreate(world);
  dMassSetZero(&m);
  dMassSetCapsuleTotal(&m,POLE_MASS,3,POLE_RADIUS, POLE_LENGTH);
  dBodySetMass(stbBody,&m);
  dBodySetPosition(stbBody, -LENGTH, 0, STARTZ);

  dMatrix3 stbR;
  dRFromAxisAndAngle(stbR, 0.0, 1.0, 0.0, M_PI/2.0);
  dBodySetRotation(stbBody, stbR);
  stbPole =  dCreateCapsule(space, POLE_RADIUS, POLE_LENGTH);
  dGeomSetBody(stbPole, stbBody);

  
   // ヒンジジョイント
  stbJoint = dJointCreateHinge(world, 0); // ヒンジジョイントの生成
  dJointAttach(stbJoint, body[0], stbBody); // 玉と円柱のボディをジョイントで結合
  dJointSetHingeAnchor(stbJoint, -LENGTH, 0, STARTZ); // ヒンジのアンカー(中心点）を設定


  // wheel bodies
  for (i=1; i<=2; i++) {
    body[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (body[i],q);
    dMassSetSphere (&m,1,RADIUS);
    dMassAdjust (&m,WMASS);
    dBodySetMass (body[i],&m);
    sphere[i-1] = dCreateSphere (0,RADIUS);
    dGeomSetBody (sphere[i-1],body[i]);
  }
  dBodySetPosition (body[1],-0.2*LENGTH, 0.5*WIDTH,STARTZ-HEIGHT*0.3);
  dBodySetPosition (body[2],-0.2*LENGTH,-0.5*WIDTH,STARTZ-HEIGHT*0.3);

  // front wheel hinge
  /*
  joint[0] = dJointCreateHinge2 (world,0);
  dJointAttach (joint[0],body[0],body[1]);
  const dReal *a = dBodyGetPosition (body[1]);
  dJointSetHinge2Anchor (joint[0],a[0],a[1],a[2]);
  dJointSetHinge2Axis1 (joint[0],0,0,1);
  dJointSetHinge2Axis2 (joint[0],0,1,0);
  */

  // front and back wheel hinges
  for (i=0; i<2; i++) {
    joint[i] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[i],body[0],body[i+1]);
    const dReal *a = dBodyGetPosition (body[i+1]);
    dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[i],1,0,1);
    dJointSetHinge2Axis2 (joint[i],0,1,0);
  }

  
  // set joint suspension
  for (i=0; i<2; i++) {
    dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
  }

  
  // lock back wheels along the steering axis
  for (i=1; i<2; i++) {
    // set stops to make sure wheels always stay in alignment
    dJointSetHinge2Param (joint[i],dParamLoStop,0);
    dJointSetHinge2Param (joint[i],dParamHiStop,0);
    // the following alternative method is no good as the wheels may get out
    // of alignment:
    //   dJointSetHinge2Param (joint[i],dParamVel,0);
    //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
  }

  // create car space and add it to the top level space
  car_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (car_space,0);
  dSpaceAdd (car_space,box[0]);
  dSpaceAdd (car_space,sphere[0]);
  dSpaceAdd (car_space,sphere[1]);

  
  // environment
  ground_box = dCreateBox (space, goalWidth, goalLength, goalHeight);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,0);
  dGeomSetPosition (ground_box, goalX, goalY, goalZ);
  dGeomSetRotation (ground_box,R);

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);
  

  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  
  return 0;
}
