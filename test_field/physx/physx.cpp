#include <stdio.h>

#include <GL\glut.h>
#include <GL\gl.h>

#include <Physics\include\NxPhysics.h>

#define NOMINMAX

//#include <windows.h>

#pragma comment(lib,"PhysXLoader.lib")
#pragma comment(lib,"NxCharacter.lib")
#pragma comment(lib,"NxCooking.lib")

#pragma comment(lib,"NxExtensions.lib")

static NxPhysicsSDK* gPhysicsSDK = NULL;

NxScene* gScene = NULL;

void initNX(void){
gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);

gPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.01);

NxSceneDesc sceneDesc;
sceneDesc.gravity.set(0.0f, -9.8f, 0.0f);
sceneDesc.simType = NX_SIMULATION_SW;

gScene = gPhysicsSDK->createScene(sceneDesc);

NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0);
defaultMaterial->setRestitution(0.5);
defaultMaterial->setStaticFriction(0.5);
defaultMaterial->setDynamicFriction(0.5);

}

void deinitNX(void){
gPhysicsSDK->releaseScene(*gScene);
gScene = 0;

gPhysicsSDK->release();

}

void display(void){
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
glLoadIdentity();

glutSwapBuffers();
}

void reshape(int w, int h){
glViewport(0, 0, (GLsizei)w, (GLsizei)h);
glMatrixMode(GL_PROJECTION);
glLoadIdentity();
gluPerspective(60, (GLfloat)w / (GLfloat)h, 0.1f, 100.0f);
glMatrixMode(GL_MODELVIEW);

}

int main(int argc, char** argv) {
glutInit(&argc, argv);
glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL);
glutInitWindowSize(500, 500);
glutInitWindowPosition(100, 100);

glutCreateWindow("Physics");

initNX();

glutDisplayFunc(display);
glutIdleFunc(display);
glutReshapeFunc(reshape);

glutMainLoop();

deinitNX();

return 0;
}
