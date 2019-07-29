#include "proj/etc.hpp"

float xy_distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
        float distance;

        distance = sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));

        return distance;
}

int fork_mouse_event(float x, float y, char * command)
{
	pid_t pid;

	pid = fork();

	if(pid == 0)	// Child Process
	{
		std::cout << "[CALL MOUSE EVENT]: "<< command << std::endl;
		if(!strcmp(command, "click"))	// strcmp return 0(false) if they are same things..
		{
			std::cout << "TEST CLICK!!" << std::endl;
			exit(0);
			//execlp();
		}
	}
	else if(pid > 0)	// Parent Process
	{
		return 1;
	}

}
