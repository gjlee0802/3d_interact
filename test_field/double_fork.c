#include <stdio.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
int func()
{
     pid_t pid1;
     pid_t pid2;
     int status;

     if (pid1 = fork()) {
             /* parent process A */
             waitpid(pid1, &status, 0);
	     return 1;
     } else if (!pid1) {
             /* child process B */
             if (pid2 = fork()) {
                     exit(0);
             } else if (!pid2) {
                     /* child process C */
                     while(1)
		     {
		     }
		     exit(0);
             } else {
                     /* error */
             }
     } else {
             /* error */
     }
}

int main()
{
	func();
}
