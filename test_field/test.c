#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

int call()
{
	pid_t pid;
	pid = fork();

	if(pid == 0)
	{
		printf("Child Process\n");
		execlp("echo", "echo", "Hello", NULL);
		perror("execlp error !");
		exit(0);
	}
	else if(pid > 0)
	{
		printf("Parent Process\n");
		return 0;
	}
}

int main(int argc, char **argv)
{
	int r;
	r = call();
	
	if(r==0)
	{
		for(int i=1000; i>0; i-- )
			printf("TEST\n");
	}
	return 0;
}
