#include "glwidget.h"
#include <QTimer>
#include <math.h>
#include <QMouseEvent>
 
GLWidget::GLWidget()
{
    xRot=yRot=zRot=0.0f;
 
    timer=new QTimer(this); //타이머 생성
    timer->setInterval(10); //0.01초 간격으로 타이머 발생
    connect(timer,SIGNAL(timeout()),this,SLOT(timerFunction())); //타이머와 함수를 연결
    timer->start(); //타이머 시작
 
 
}
 
GLWidget::~GLWidget() //소멸자
{
    delete timer; //프로그램 종료할때 타이머 제거
}
 
void GLWidget::timerFunction()
{
    xRot+=0.9f; //0.01초마다 xRot 변수에 0.9f 씩 더함
    yRot+=0.8f;
 
 
    updateGL(); //GL 업데이트
}
 
void GLWidget::initializeGL() //GL 초기화
{
    glEnable(GL_DEPTH_TEST); //깊이값 반영
}
 
void GLWidget::resizeGL(int w, int h) //위젯 크기 변했을때
{
    GLfloat aspectRatio; //화면 비율
 
    if(h==0)
        h=1;
 
    glViewport(0,0,w,h); //뷰포트 설정
 
    glMatrixMode(GL_PROJECTION); //행렬모드지정
    glLoadIdentity(); //좌표계 초기화
 
    aspectRatio=(GLfloat)w/(GLfloat)h; //화면비율 지정
    if(w<=h)
    {
        windowWidth=1;
        windowHeight=1/aspectRatio;
        glFrustum(-1.0,1.0,-windowHeight,windowHeight,5.0,300.0);//3D
        //glOrtho(-100.0,100.0,-windowHeight,windowHeight,1.0,-1.0);//2D
    }
    else
    {
        windowWidth=1*aspectRatio;
        windowHeight=1;
        glFrustum(-windowWidth,windowWidth,-1.0,1.0,5.0,3000.0);
        //glOrtho(-windowWidth,windowWidth,-100.0,100.0,1.0,-1.0);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::paintGL() //실제 그리는 함수
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); //화면을 지워준다.
 
 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f,0.0f,-1500.0f); //move along z-axis// 카메라 이동
    glRotatef(yRot,0.0,1.0,0.0); //rotate 30 degress around y-axis
    glRotatef(xRot,1.0,0.0,0.0); //rotate 15 degress around x-axis
 
 
    /* create 3D-Cube */
    glBegin(GL_QUADS); //사각형
 
        //front
        glColor3f(1.0,0.0,0.0);
 
        glVertex3f(101.0,101.0,101.0);
        glVertex3f(-101.0,101.0,101.0);
        glVertex3f(-101.0,-101.0,101.0);
        glVertex3f(101.0,-101.0,101.0);
 
 
        //back
 
        glColor3f(0.0,1.0,0.0);
 
        glVertex3f(101.0,101.0,-101.0);
        glVertex3f(-101.0,101.0,-101.0);
        glVertex3f(-101.0,-101.0,-101.0);
        glVertex3f(101.0,-101.0,-101.0);
 
 
        //top
        glColor3f(0.0,0.0,1.0);
 
        glVertex3f(-101.0,101.0,101.0);
        glVertex3f(101.0,101.0,101.0);
        glVertex3f(101.0,101.0,-101.0);
        glVertex3f(-101.0,101.0,-101.0);
 
 
        //bottom
        glColor3f(0.0,1.0,1.0);
 
        glVertex3f(101.0,-101.0,101.0);
        glVertex3f(101.0,-101.0,-101.0);
        glVertex3f(-101.0,-101.0,-101.0);
        glVertex3f(-101.0,-101.0,101.0);
 
        //right
        glColor3f(1.0,0.0,1.0);
 
        glVertex3f(101.0,101.0,101.0);
        glVertex3f(101.0,-101.0,101.0);
        glVertex3f(101.0,-101.0,-101.0);
        glVertex3f(101.0,101.0,-101.0);
 
 
        //left
        glColor3f(1.0,1.0,0.0);
 
        glVertex3f(-101.0,101.0,101.0);
        glVertex3f(-101.0,-101.0,101.0);
        glVertex3f(-101.0,-101.0,-101.0);
        glVertex3f(-101.0,101.0,-101.0);
 
 
    glEnd();
}

static void qNormalizeAngle(int &angle)
{
     while (angle < 0)
         angle += 360 * 16;
     while (angle > 360 * 16)
         angle -= 360 * 16;
}
 
 
void GLWidget::setXRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != xRot) {
         xRot = angle;
         emit xRotationChanged(angle);
         updateGL();
     }
 }
 
 void GLWidget::setYRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != yRot) {
         yRot = angle;
         emit yRotationChanged(angle);
         updateGL();
     }
 }
 
 void GLWidget::setZRotation(int angle)
 {
     qNormalizeAngle(angle);
     if (angle != zRot) {
         zRot = angle;
         emit zRotationChanged(angle);
         updateGL();
     }
 }

void GLWidget::mousePressEvent(QMouseEvent *event)
 {
     lastPos = event->pos();
 }
 
 void GLWidget::mouseMoveEvent(QMouseEvent *event)
 {
     int dx = event->x() - lastPos.x();
     int dy = event->y() - lastPos.y();
 
     if (event->buttons() & Qt::LeftButton) {
         setXRotation(xRot + 1 * dy);
         setYRotation(yRot + 1 * dx);
     } else if (event->buttons() & Qt::RightButton) {
         setXRotation(xRot + 1 * dy);
         setZRotation(zRot + 1 * dx);
     }
     lastPos = event->pos();
}
 
