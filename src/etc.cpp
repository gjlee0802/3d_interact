#include "proj/etc.hpp"

//------------------------------------------------------------------------
//모니터 해상도에 따라 변경
#define Screen_width 1920
#define Screen_x_half 960
#define Screen_height 1080
#define Screen_y_half 540

//아래의 상수는 직접 측정한 MIN, MAX 값을 바탕으로 직접 계산하여 정한다.
#define Cloud_x_center -0.272605
#define Cloud_y_center 0.05739
#define Cloud_width 8.55113
#define Cloud_height 6.62856
//------------------------------------------------------------------------

pcl::PointXYZ xy_past;
pcl::PointXYZ xy_now;

float xy_distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
        float distance;

        distance = sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));

        return distance;
}

int fork_mouse_event(float x, float y, char * command)
{
	char x_buff[256];
	char y_buff[256];

	// Change to moniter's coordinates (This should be changed later! NOT FINISHED)
	x = Screen_x_half + (x-Cloud_x_center)*(Screen_width/Cloud_width);
	y = Screen_y_half + (y*(-1)-Cloud_y_center)*(Screen_height/Cloud_height);

	sprintf(x_buff, "%f", x);
	sprintf(y_buff, "%f", y);

	pid_t pid;

	pid = fork();

	if(pid == 0)	// Child Process
	{
		std::cout << "[CALL MOUSE EVENT]: "<< command << std::endl;

		if(!strcmp(command, "move"))
		{
			std::cout << "[xdotool MOVE]: "<<"("<<x<<", "<<y<<")"<<std::endl;
			execlp("xdotool", "xdotool", "mousemove", x_buff, y_buff, NULL);
			perror("execlp error!");
			exit(0);
		}

		if(!strcmp(command, "click"))	// strcmp return 0(false) if they are same things..
		{
			std::cout << "[xdotool CLICK]: "<<"("<<x<<", "<<y<<")"<< std::endl;
			execlp("xdotool", "xdotool", "click", "1", NULL);
			perror("execlp error!");
			exit(0);
		}

		if(!strcmp(command, "scroll_up"))
		{
			std::cout << "[xdotool SCROLL_UP]" << std::endl;
			execlp("xdotool", "xdotool", "click", "4", NULL);
			perror("execlp error!");
			exit(0);
		}

		if(!strcmp(command, "scroll_down"))
		{
			std::cout << "[xdotool SCROLL_DOWN]" << std::endl;
			execlp("xdotool", "xdotool", "click", "5", NULL);
			perror("execlp error!");
			exit(0);
		}
		
		if(!strcmp(command, "mouse_down"))
		{
			std::cout << "[xdotool MOUSE_DOWN]" << std::endl;
			execlp("xdotool", "xdotool", "mousedown", "1", NULL);
			perror("execlp error!");
			exit(0);
		}

		if(!strcmp(command, "mouse_up"))
		{
			std::cout << "[xdotool MOUSE_UP]" << std::endl;
			execlp("xdotool", "xdotool", "mouseup", "1", NULL);
			perror("execlp error!");
			exit(0);
		}

	}
	else if(pid > 0)	// Parent Process
	{
		return 1;
	}

}
