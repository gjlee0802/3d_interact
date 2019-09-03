#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/input.h>

int main()
{
	struct input_event event, event_end;
	
	int fd = open("/dev/input/event3", O_RDWR);
	
	int i;

	if(!fd)
	{
		printf("open error:%s\n", strerror(errno));
		return -1;
	}
	
	memset(&event, 0, sizeof(event));
	memset(&event_end, 0, sizeof(event_end));

	gettimeofday(&event.time, NULL);
	event.type = EV_REL;
	event.code = REL_WHEEL;
	event.value = 1;
	
	//EV_SYN : 구분을 위한 이벤트
	gettimeofday(&event_end.time, NULL);
	event_end.type = EV_SYN;
	event_end.code = SYN_REPORT;
	event_end.value  = 0;
	for(i=0; i<5; i++)
	{
		write(fd, &event, sizeof(event));
		write(fd, &event_end, sizeof(event_end));
		sleep(1);
	}

	close(fd);
	return 0;
}
