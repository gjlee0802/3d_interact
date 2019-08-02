#include "proj/etc.hpp"

//------------------------------------------------------------------------
//모니터 해상도에 따라 변경
#define Screen_width 1920
#define Screen_x_half 960	// Screen_width/2 
#define Screen_height 1080
#define Screen_y_half 540	// Screen_height/2

//아래의 상수는 직접 측정한 MIN, MAX 값을 바탕으로 직접 계산하여 정한다.
#define Cloud_x_center -0.272605
#define Cloud_y_center 0.05739
#define Cloud_width 8.55113
#define Cloud_height 6.62856
//------------------------------------------------------------------------

pcl::PointXYZ mouse_past;
pcl::PointXYZ mouse_now;

/* 
 * xy_distance():
 * pcl::PointXYZ (x, y, z가 담겨있는 구조체) 파라미터로 받는다.
 * z를 무시하고 x, y 좌표만을 고려하여 2차원 xy평면 상에서의 두 점 사이의 거리를 계산한다.
 * 계산한 거리를 반환한다.
 */
float xy_distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
        float distance;

        distance = sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));

        return distance;
}

/*
 * fork_mouse_event(): 
 * fork()를 통해 자식 프로세스를 낳는다.
 * 자식 프로세스에서는 exec계열의 함수를 통해 xdotool을 실행한다.
 * 부모 프로세스에서는 pid_wait()를 통해 자식 프로세스가 종료될 때까지 기다린다.
 * fork()를 실패하면 -1을 반환한다.
 * command 인식에 실패하면 0을 반환한다.
 * 성공하면 1을 반환한다.
 */
int fork_mouse_event(float x, float y, char * command)
{
	char x_buff[256];
	char y_buff[256];

	float mouse_dis;

	// Change to moniter's coordinates (This should be changed later! NOT FINISHED)	// 화면 해상도에 따른 마우스 좌표 설정 (비율 변환)
	x = Screen_x_half + (x-Cloud_x_center)*(Screen_width/Cloud_width);
	y = Screen_y_half + (y*(-1)-Cloud_y_center)*(Screen_height/Cloud_height);

	sprintf(x_buff, "%f", x);
	sprintf(y_buff, "%f", y);

	// mouse_dis filtering START: 마우스 흔들림을 줄이기 위함
	mouse_now.x = x; mouse_now.y = y;

	std::cout << "mouse_past: " << mouse_past.x << ", "<< mouse_past.y << std::endl;
	std::cout << "mouse_now: " << mouse_now.x << ", "<< mouse_now.y <<std::endl;

	mouse_dis = xy_distance(mouse_past, mouse_now);
	std::cout << "mouse_dis: "<< mouse_dis << std::endl;

	if(mouse_dis < 25.0 && mouse_dis > 4.0)
	{
		return 1;
	}
	else
	{
		std::cout << "TEST" << std::endl;
	}
	mouse_past = mouse_now;
	// mouse_dis filtering END
	
	int status;
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
			exit(1);
		}

		else if(!strcmp(command, "click"))	// strcmp return 0(false) if they are same things..
		{
			std::cout << "[xdotool CLICK]: "<<"("<<x<<", "<<y<<")"<< std::endl;
			execlp("xdotool", "xdotool", "click", "1", NULL);
			perror("execlp error!");
			exit(1);
		}

		else if(!strcmp(command, "scroll_up"))
		{
			std::cout << "[xdotool SCROLL_UP]" << std::endl;
			execlp("xdotool", "xdotool", "click", "4", NULL);
			perror("execlp error!");
			exit(1);
		}

		else if(!strcmp(command, "scroll_down"))
		{
			std::cout << "[xdotool SCROLL_DOWN]" << std::endl;
			execlp("xdotool", "xdotool", "click", "5", NULL);
			perror("execlp error!");
			exit(1);
		}
		
		else if(!strcmp(command, "mouse_down"))
		{
			std::cout << "[xdotool MOUSE_DOWN]" << std::endl;
			execlp("xdotool", "xdotool", "mousedown", "1", NULL);
			perror("execlp error!");
			exit(1);
		}

		else if(!strcmp(command, "mouse_up"))
		{
			std::cout << "[xdotool MOUSE_UP]" << std::endl;
			execlp("xdotool", "xdotool", "mouseup", "1", NULL);
			perror("execlp error!");
			exit(1);
		}
		else
		{
			std::cout << "[MOUSE EVENT]: UNKNOWN command!: "<< command << std::endl;
			exit(0);
		}

	}
	else if(pid > 0)	// Parent Process get child process's PID
	{
		//waitpid(pid, NULL, 0);
		std::cout << "Parent: wait "<< pid << std::endl;

		waitpid(pid, &status, 0);
		if(WIFEXITED(status))
		{
			std::cout << "Child process killed" << std::endl;
			return 1;
		}

	}
	else if(pid < -1)
	{
		return -1;
	}

}

/* 
 * detect_mode(): 
 * 센서의 정보에 따라 char *mode를 수정한다.
 * 아두이노로부터 센서 정보를 받온 이후에 호출해야 힌디.
 * 아두이노로부터 받은 센서 정보를 2X2행렬의 파라미터로 받는다.
 * 성공시 1, 실패시 0을 반환한다.
 */
int detect_mode(char *mode, int (*pressed_finger)[2]) 
{
        // 아무것도 눌리지 않았을 때
        if(pressed_finger[0][0]==false
        && pressed_finger[0][1]==false
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==false)
		strcpy(mode, "nothing");
        	
       // 오직 하나의 검지 손가락만 눌렸을 때?? 어떤 상황에서 pick_hold 모드로 들어갈>지는 향후에 변경
        else if((pressed_finger[0][0]==true
        && pressed_finger[0][1]==false
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==false)
        || (pressed_finger[0][0]==false
        && pressed_finger[0][1]==false
        && pressed_finger[1][0]==true
        && pressed_finger[1][1]==false))
        	strcpy(mode, "pick_hold");
        // 두 손의 중지 손가락이 모두 눌렸을 때
        else if(pressed_finger[0][0]==false
        && pressed_finger[0][1]==true
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==true)
                strcpy(mode, "zoom_scroll");
        else
        {
                        std::cout << "[?]: Cannot detect the mode!" << std::endl;
			return 0;
	}

	return 1;
}
