#include "proj/etc.hpp"

/* 
 * xy_distance():
 * 
 * First/Second Parameter: 거리를 계산할 두 점 데이터 pcl::PointXYZ (x, y, z가 담겨있는 구조체)를 받는다.
 *
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
 * fork_xdotool_event(): 
 *
 * Fist Parameter: 스크린의 정보가 담긴 Screen_data구조체의 주소를 받는다.
 * Second/Third Parameter: Point Cloud Data 기준의 (x,y)좌표를 받는다.
 * Fourth Parameter: 실행할 명령을 입력받는다.
 *
 * double fork를 통해 좀비 프로세스가 발생하지 않도록 한다.
 * 
 * 부모 프로세스A는 fork()를 통해 자식 프로세스B를 낳고 자식 프로세스가 종료하길 기다린다.
 * 자식 프로세스B에서는 fork()를 통해 자식 프로세스C를 낳고 exit()를 실행하여 바로 종료한다.
 * C의 부모 프로세스였던 프로세스B가 종료되었으므로 init소속의 프로세스가 되어 exec함수가 종료된 후에 정상종료한다.
 * (exec계열의 함수를 통해 마우스 제어 프로그램인 xdotool을 실행한다.)
 * 
 * -> double fork를 안하고 단일 fork를 한다면 scroll과 같은 xdotool실행에서 시간이 상당히 소요되어 프레임 속도가 매우 저하됨.
 *
 * fork()를 실패하면 -1을 반환한다.
 * command 인식에 실패하면 0을 반환한다.
 * 성공하면 1을 반환한다.
 */

pcl::PointXYZ mouse_past;
pcl::PointXYZ mouse_now;

int fork_xdotool_event(struct Screen_data *sd, float x, float y, char * command)
{
	char x_buff[256];
	char y_buff[256];

	float mouse_dis;


	// Change to moniter's coordinates (This should be changed later! NOT FINISHED)	// 화면 해상도에 따른 마우스 좌표 설정 (비율 변환)
	x = sd->Screen_x_half + (x-sd->Cloud_x_center)*(sd->Screen_width/sd->Cloud_width);
	y = sd->Screen_y_half + (y*(-1)-sd->Cloud_y_center)*(sd->Screen_height/sd->Cloud_height);

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
	pid_t pid1;	// Child Process B
	pid_t pid2;

	pid1 = fork();

	if(pid1 == 0)	// Child Process B
	{
		pid2 = fork();

		if(pid2 == 0)	// Child Process C
		{
			
			std::cout << "[CALL xdotool EVENT]: "<< command << std::endl;

			if(!strcmp(command, "move"))
			{
				std::cout << "[xdotool MOVE]: "<<"("<<x<<", "<<y<<")"<<std::endl;
				execlp("xdotool", "xdotool", "mousemove", x_buff, y_buff, NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "click"))	// strcmp return 0(false) if they are same things..
			{
				std::cout << "[xdotool CLICK]: "<<"("<<x<<", "<<y<<")"<< std::endl;
				execlp("xdotool", "xdotool", "click", "1", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "scroll_up"))
			{
				std::cout << "[xdotool SCROLL_UP]" << std::endl;
				execlp("xdotool", "xdotool", "click", "4", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "scroll_down"))
			{
				std::cout << "[xdotool SCROLL_DOWN]" << std::endl;
				execlp("xdotool", "xdotool", "click", "5", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "mouse_down"))
			{
				std::cout << "[xdotool MOUSE_DOWN]" << std::endl;
					execlp("xdotool", "xdotool", "mousedown", "1", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "mouse_up"))
			{
				std::cout << "[xdotool MOUSE_UP]" << std::endl;
				execlp("xdotool", "xdotool", "mouseup", "1", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}

			else if(!strcmp(command, "key_Up"))
			{
				std::cout << "[xdotool KEY]: up" << std::endl;
				execlp("xdotool", "xdotool", "key", "--repeat", "5", "Up", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else if(!strcmp(command, "key_Left"))
			{
				std::cout << "[xdotool KEY]: a" << std::endl;
				execlp("xdotool", "xdotool", "key", "--repeat", "5", "Left", NULL);
				perror("[ERROR]: execlp");
				exit(1);
				
			}
			else if(!strcmp(command, "key_Down"))
			{
				std::cout << "[xdotool KEY]: s" << std::endl;
				execlp("xdotool", "xdotool", "key", "--repeat", "5", "Down", NULL);
				perror("[ERROR]: execlp");
				exit(1);
				
			}
			else if(!strcmp(command, "key_Right"))
			{
				std::cout << "[xdotool KEY]: d" << std::endl;
				execlp("xdotool", "xdotool", "key", "--repeat", "5", "Right", NULL);
				perror("[ERROR]: execlp");
				exit(1);
			}
			else
			{
				std::cout << "[xdotool WARNING]: Command not found!: "<< command << std::endl;
				exit(0);
			}



		}
		else if(pid2 > 0)	// Child Process B
		{
			exit(0);
		}
		else
		{
			/*error*/
			std::cout << "[ERROR]: fork" << std::endl;
			return -1;
		}

	}
	else if(pid1 > 0)	// Parent Process get child process's PID
	{

		std::cout << "Parent: wait Child Process"<< "("<<pid1<<")" << std::endl;
		waitpid(pid1, &status, 0);
		if(WIFEXITED(status))
		{
			std::cout << "Child process killed" << std::endl;
			return 1;
		}

	}
	else if(pid1 < -1)
	{
		std::cout << "[ERROR]: fork" << std::endl;
		return -1;
	}

}

/* 
 * detect_mode():
 *
 * First Parameter: 변경될 mode Character Pointer을 받는다.
 * Second Parameter: 손가락 센서의 정보가 담긴 Array 의 주소를 받는다.
 *
 * 센서의 정보에 따라 char *mode를 수정한다.
 * 아두이노로부터 센서 정보를 받온 이후에 호출해야 힌디.
 * 아두이노로부터 받은 센서 정보를 2X2행렬의 파라미터로 받는다.
 *
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
        	
       // 오직 하나의 중지만 눌렸을 때
        else if((pressed_finger[0][0]==false
        && pressed_finger[0][1]==true
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==false)
        || (pressed_finger[0][0]==false
        && pressed_finger[0][1]==false
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==true))
        	strcpy(mode, "pick_hold");

        // 두 손의 중지가 모두 눌렸을 때
        else if(pressed_finger[0][0]==false
        && pressed_finger[0][1]==true
        && pressed_finger[1][0]==false
        && pressed_finger[1][1]==true)
                strcpy(mode, "zoom_scroll");
	
	// 오직 하나의 검지만 눌렸을 때
	else if((pressed_finger[0][0]==true
	&& pressed_finger[0][1]==false
	&& pressed_finger[1][0]==false
	&& pressed_finger[1][1]==false)
	|| (pressed_finger[0][0]==false
	&& pressed_finger[0][1]==false
	&& pressed_finger[1][0]==true
	&& pressed_finger[1][1]==false))
		strcpy(mode, "click_once");


        else
        {
                        std::cout << "[?]: Cannot detect the mode!" << std::endl;
			return 0;
	}

	return 1;
}

/*
 * This is the method that handles the mouse events. 
 * Every time any kind of mouse event is registered, this function will be called. 
 * In order to see exactly what that event is, we need to extract that information from the event instance.
 */

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void *key_id_void)
{
  char *key_id = static_cast<char *> (key_id_void);

  if (event.getKeySym () == "t" && event.keyDown ())
  {
	  std::cout << "[KEY]: t was pressed" << std::endl;
	  strcpy(key_id, "key_t");
  }
  else if (event.getKeySym () == "y" && event.keyDown())
  {
	  std::cout << "[KEY]: y was pressed" << std::endl;
	  strcpy(key_id, "key_y");
  }

}
/*
 *The same approach applies for the keyboard events.
 */
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void *viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
	  std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
  }
}

