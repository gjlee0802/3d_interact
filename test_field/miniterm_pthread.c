#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>


#define MODEM_DEV	"/dev/rfcomm0"
#define BAUDRATE	9600


void *thread_func(void *);

struct data_pkg
{
	int fd;
};
typedef struct data_pkg arg;

pthread_t init_miniterm()
{
	int fd;
	pthread_t tid;
	struct termios oldtio, newtio, oldstdtio, newstdtio;

	fd = open(MODEM_DEV, O_RDWR | O_NOCTTY);

	if(fd < 0){perror(MODEM_DEV); exit(-1);}
	/*
	tcgetattr(fd, &oldtio);

	newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

	newtio.c_iflag = IGNPAR;

	newtio.c_oflag = 0;

	newtio.c_lflag=0;

	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] =0;
	
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
	
	tcsetattr(1, TCSANOW, &newtio);
	*/
	/*
	tcgetattr(0,&oldstdtio);
	tcgetattr(0,&newstdtio);
	newstdtio.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(0,TCSANOW,&newstdtio);
	*/
	
	
	arg* data = malloc(sizeof(arg));
	
	data->fd = fd;
	
	if(pthread_create(&tid, NULL, thread_func, (void *)data)==-1)
	{
		fprintf(stdout, "ERROR : pthread_create\n");

		tcsetattr(fd, TCSANOW, &oldtio);
		close(fd);
		exit(-1);
	}

	while(1)
	{

	}

	return tid;
}

int main(void)
{
	pthread_t tid;
	tid = init_miniterm();
	pthread_join(tid,NULL);
	return 0;
}

void *thread_func(void *data)
{
	
	arg *my_data = (arg *)data;
	char c;

	while(1)
	{
		read(my_data->fd, &c, 1);
	
		//if(c == ' ')
		printf("%c",c);
		if(c == '\0')
			write(1, (char *)'N', 1);
	}
}
