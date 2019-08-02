#ifndef ETC_H
#define ETC_H

#include <cmath>	// for calculating xy_distance
#include <pcl/point_types.h>

// for fork(), exec()
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

// for strcpy()
#include <string.h>

float xy_distance(pcl::PointXYZ, pcl::PointXYZ);

int fork_mouse_event(float, float, char *);

int detect_mode(char *, int (* )[2]);

#endif
