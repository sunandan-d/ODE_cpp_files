// This is a practice code for a robot arm simulated with Open Dynamics Engine (ODE)

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  
#endif

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#endif
#define NUM 4                          

dWorldID      world;                   
dSpaceID      space;                   
dGeomID       ground;                  
dJointGroupID contactgroup;            
dJointID      joint[NUM];              
dsFunctions   fn;  

typedef struct {
  dBodyID body;                        
  dGeomID geom;                        
} MyObject;                         

MyObject rlink[NUM];                   
dReal  THETA[NUM] = {0.0};             


void  makeArm()
{
  dMass mass;                                    
  dReal x[NUM]      = {0.00, 0.00, 0.00, 0.00}; 
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  
  dReal z[NUM]      = {0.05, 0.50, 1.50, 2.50};  
  dReal length[NUM] = {0.10, 0.90, 1.00, 1.00};  
  dReal weight[NUM] = {9.00, 2.00, 2.00, 2.00};  
  dReal r[NUM]      = {0.20, 0.04, 0.04, 0.04};  
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};   
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};
  dReal c_z[NUM]    = {0.00, 0.10, 1.00, 2.00};  
  dReal axis_x[NUM] = {0, 0, 0, 0};              
  dReal axis_y[NUM] = {0, 0, 1, 1};              
  dReal axis_z[NUM] = {1, 1, 0, 0};              


  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass,weight[i],3,r[i],length[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateCapsule(space,r[i],length[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }

  
  joint[0] = dJointCreateFixed(world, 0);  
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);
  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0);
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
}


void drawArm()
{
   dReal r,length;

   for (int i = 0; i < NUM; i++ ) {       
     dGeomCapsuleGetParams(rlink[i].geom, &r,&length);
     dsDrawCapsule(dBodyGetPosition(rlink[i].body),
     dBodyGetRotation(rlink[i].body),length,r);
   }
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  10.0, fMax = 100.0;     

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);   
    dReal z = THETA[j] - tmp;                     
    dJointSetHingeParam(joint[j],dParamVel, k*z);  
    dJointSetHingeParam(joint[j],dParamFMax,fMax); 
  }
}


void start()
{
  float xyz[3] = {    3.0f, 1.3f, 0.8f};          
  float hpr[3] = { -160.0f, 4.5f, 0.0f};         
  dsSetViewpoint(xyz, hpr);                      
}


void command(int cmd)
{
  switch (cmd) {
    case 'j': THETA[1] += M_PI/180; break;    
    case 'f': THETA[1] -= M_PI/180; break;     
    case 'k': THETA[2] += M_PI/180; break;   
    case 'd': THETA[2] -= M_PI/180; break;   
    case 'l': THETA[3] += M_PI/180; break;   
    case 's': THETA[3] -= M_PI/180; break;   
  }
}


void simLoop(int pause)
{
  Pcontrol();                                  
  dWorldStep(world, 0.01);                     
  drawArm();                                  
}


void setDrawStuff()
{
  fn.version = DS_VERSION;                    
  fn.start   = &start;                         
  fn.step    = &simLoop;                      
  fn.command = &command;                       
  fn.path_to_textures = "../../drawstuff/textures";
}

int main(int argc, char **argv)
{
  dInitODE();                                     
  setDrawStuff();
  world        = dWorldCreate();                 
  space        = dHashSpaceCreate(0);             
  contactgroup = dJointGroupCreate(0);            
  ground       = dCreatePlane(space, 0, 0, 1, 0); 
  dWorldSetGravity(world, 0, 0, -9.8);            
  makeArm();                                      
  dsSimulationLoop(argc, argv, 640, 480, &fn);    
  dSpaceDestroy(space);                           
  dWorldDestroy(world);                           
  dCloseODE();                                    
  return 0;
}
