#include "proj/etc.hpp"

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
	x = 910 + (x*100*2);
	y = 540 + (y*100*2)*(-1);
	// Casting: (int) to (char *)
	sprintf(x_buff, "%d", (int)x);
	sprintf(y_buff, "%d", (int)y);

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
		
	}
	else if(pid > 0)	// Parent Process
	{
		return 1;
	}

}
