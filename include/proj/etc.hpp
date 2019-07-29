#include <cmath>	// for calculating xy_distance
#include <pcl/point_types.h>

// for fork(), exec()
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

float xy_distance(pcl::PointXYZ, pcl::PointXYZ);

int fork_mouse_event(float, float, char *);
