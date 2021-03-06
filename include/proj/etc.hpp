#ifndef ETC_H
#define ETC_H

#include <cmath>	// for calculating xy_distance
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/input.h>
#include <errno.h>
#include <pthread.h>


extern int pressed_finger[2][2];

struct Gyro{
	float x;
	float y;
	float z;
};

extern struct Gyro gyro;

struct Screen_data {
        //모니터 해상도에 따라 변경
        float Screen_width   = 1920;		// 모니터 화면의 x좌표 최댓값
        float Screen_height  = 1080;		// 모니터 화면의 y좌표 최댓값
        float Screen_x_half  = Screen_width/2;	
        float Screen_y_half  = Screen_height/2;

        // If your touch area has changed, you should change these with estimated MIN, MAX..
        float MAX_X = 5.05075;
        float MAX_Y = 4.19409;
        float MIN_X = -5.58617;
        float MIN_Y = -4.05131;

        float Cloud_x_center = (MAX_X+MIN_X)/2;
        float Cloud_y_center = (MAX_Y+MIN_Y)/2;
        float Cloud_width    = (MAX_X-MIN_X);
        float Cloud_height   = (MAX_Y-MIN_Y);
};

struct WindowInputs {
	char key_id[32];
};

float xy_distance(pcl::PointXYZ, pcl::PointXYZ);

int input_event();

int fork_xdotool_event(struct Screen_data *, float, float, char *);

int detect_mode(char *, int (* )[2]);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &, void *);
void mouseEventOccurred(const pcl::visualization::MouseEvent &, void *);

extern int read_num;

extern int issame;

void *pthread_create(void *);
void *init_miniterm();

int fork_unity();

// 문자열 처리 함수
void char_append(char *, char);

#endif
