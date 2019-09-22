#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main()
{
	char buff[10]="123456789";
	printf("%s\n", buff);
	buff[0] = '\0';
	printf("%s\n", buff);
	printf("%c\n", buff[1]);
}
