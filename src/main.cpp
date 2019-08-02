#include <iostream>

#include <queue>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>			// for K-means Cluster Extraction

#include <boost/thread.hpp>

#include "proj/etc.hpp"

//----------------------------------------------------------
/*
Estimate MIN MAX 결과에 따라 변경해줘야 함!!
etc.cpp의 define 또한 변경이 필요함.
*/

//#define ESTIMATE_MIN_MAX			// 만약 측정하고 싶다면 주석을 해제하고 출력을 확인

#define MAX_X 4.00296	//<---MAX.x
#define MAX_Y 3.37167	//<---MAX.y
#define MIN_X -4.54817	//<---MIN.x
#define MIN_Y -3.25689	//<---MIN.y
//----------------------------------------------------------
//

typedef pcl::PointXYZ PointT;
using namespace std;


class GestureHandler
{
public:

	int pressed_finger[2][2]={{0,0},{0,0}}; // This should be changed simultaneously by arduino's informations
        int (*pressed_finger_Ptr)[2];

        char mode[1024];

        int exe_once=0;

        queue<float> dis;       // Save distances between centr1 and centr2 in this vector container

	//Constructor(생성자)
	GestureHandler () : 
		viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"))
	{
		pressed_finger_Ptr = pressed_finger;
		detect_mode(mode, pressed_finger_Ptr);	// 함수 2번째 파라미터에 2차원 포인터 변수를 넘겨준다.
	}

	void viewer_set()
	{
		viewer->setFullScreen(false);
		viewer->setSize(1280, 960);
		//viewer->addCoordinateSystem(1.0);
		viewer->setCameraPosition(0,0,-6.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0);
	}

	void draw_max_min_line(float z_max)
	{
		pcl::PointXYZ a;
		pcl::PointXYZ b;

		a.x = MIN_X;
		a.y = (MIN_Y+MAX_Y)/2;
		a.z = z_max;
		
		b.x = MAX_X;
		b.y = (MIN_Y+MAX_Y)/2;
		b.z = z_max;
		
		viewer->addLine(a, b, 0, 0, 1.0, "Cloud_width");
		
		a.x = (MIN_X+MAX_X)/2;
		a.y = MIN_Y;
		a.z = z_max;
		
		b.x = (MIN_X+MAX_X)/2;
		b.y = MAX_Y;
		b.z = z_max;
		
		viewer->addLine(a, b, 0, 0, 1.0, "Cloud_height");
	}

	void draw_tp_box(float z_min, float z_max, double _r,  double _g, double _b)
	{
		pcl::PointXYZ a;
		pcl::PointXYZ b;

		a.x = MIN_X;
		a.y = MAX_Y;
		a.z = z_max;
		b.x = MAX_X;
		b.y = MAX_Y;
		b.z = z_max;
		viewer->addLine(a, b, _r,_g,_b, "line_x_1");

		a.x = MIN_X;
                a.y = MIN_Y;
                a.z = z_max;
                b.x = MAX_X;
                b.y = MIN_Y;
                b.z = z_max;
		viewer->addLine(a, b, _r,_g,_b, "line_x_2");

		a.x = MIN_X;
                a.y = MIN_Y;
                a.z = z_min;
                b.x = MAX_X;
                b.y = MIN_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_x_3");

		a.x = MIN_X;
                a.y = MAX_Y;
                a.z = z_min;
                b.x = MAX_X;
                b.y = MAX_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_x_4");

		a.x = MIN_X;
                a.y = MAX_Y;
                a.z = z_max;
                b.x = MIN_X;
                b.y = MIN_Y;
                b.z = z_max;
                viewer->addLine(a, b, _r,_g,_b, "line_y_1");

		a.x = MAX_X;
                a.y = MIN_Y;
                a.z = z_max;
                b.x = MAX_X;
                b.y = MIN_Y;
                b.z = z_max;
                viewer->addLine(a, b, _r,_g,_b, "line_y_2");

		a.x = MAX_X;
                a.y = MAX_Y;
                a.z = z_min;
                b.x = MAX_X;
                b.y = MIN_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_y_3");

		a.x = MIN_X;
                a.y = MAX_Y;
                a.z = z_min;
                b.x = MIN_X;
                b.y = MIN_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_y_4");

		a.x = MIN_X;
                a.y = MAX_Y;
                a.z = z_min;
                b.x = MAX_X;
                b.y = MIN_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_d_min1");

		a.x = MAX_X;
                a.y = MAX_Y;
                a.z = z_min;
                b.x = MIN_X;
                b.y = MIN_Y;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_d_min2");

	}


	void run ()
	{
		boost::mutex mutex;

		//-------------------------------------
		float touch_box_min_z=8.0;
		float touch_box_max_z=8.2;
		float cloud_filtered_max_z = 13.0;
		//-------------------------------------

		pcl::PointXYZ tempPt;
		pcl::PointXYZ MAX; pcl::PointXYZ MIN;	// Use for estimating MAX values or MIN values of the coordinates.

		pcl::PointCloud<PointT>::ConstPtr cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_touch (new pcl::PointCloud<PointT>);

		std::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
		[&cloud, &mutex](const pcl::PointCloud<PointT>::ConstPtr &ptr ){
			boost::mutex::scoped_lock lock( mutex );
			cloud = ptr->makeShared();
		};

		// Initialize Grabber Pointer ("interface")
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(
                "", pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz,
                pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz);// QVGA_60Hz Recommended (VGA_30Hz or QVGA_60Hz)


		interface->registerCallback (f);

		interface->start ();

		// Create the PassThrough Filtering object
		pcl::PassThrough<PointT> pass;

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
			boost::mutex::scoped_try_lock lock( mutex );
			if( lock.owns_lock())
			{
				std::cout << "-----------------------------------------------------------" << std::endl;

				detect_mode(mode, pressed_finger);
				std::cout << "[detect_mode]: "<< mode << std::endl;

				// PassThrough Filtering START
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(touch_box_min_z, cloud_filtered_max_z); //QVGA 기준 || 0.0~1.0?? in VGA
				pass.filter(*cloud_filtered);

				pass.setInputCloud(cloud_filtered);
				pass.setFilterLimits(touch_box_min_z, touch_box_max_z);
				pass.filter(*cloud_touch);	// This cloud_touch will be in Touch Box Field
				// PassThrough Filtering END

				
				// Remove shapes to update shapes START
				std::cout << "[REMOVE SHAPE]: ALL" << std::endl;
				viewer->removeAllShapes();
				// Reomve shapes END
				
				
				// Get_max_and_min_coordinates
				// Get Z Minimum Point
				/*
                                z_minPt는 z값이 가장 작은 점의 xyz좌표를 담는 구조체,
                                maxPt는 가장 큰 x, 가장 큰 y, 가장 큰 z를 각각 담는 구조체이다.
                                */
				pcl::PointXYZ z_minPt, maxPt;	

                                pcl::getMinMax3D (*cloud_filtered, z_minPt, maxPt);

                                std::cout << "Min z: " << z_minPt.z << std::endl;

				for(size_t i=0; i < cloud_filtered->points.size(); i++)
				{
					if(cloud_filtered->points[i].z == z_minPt.z)
					{
						z_minPt.x = cloud_filtered->points[i].x;
						z_minPt.y = cloud_filtered->points[i].y;
						break;
					}
				}
				std::cout << "z_minPt: ("<< z_minPt.x <<", "<< z_minPt.y <<")"<<std::endl;

			#ifdef ESTIMATE_MIN_MAX
				// ---------------------------------------------------------Estimate MIN MAX START
                                        /* 
					화면 비율과 좌표를 맞추기 위해 cloud_touch의 MIN, MAX를 측정한다.
                                        PointCloud 상의 최대 x, y 좌표를 측정하여 TouchBox의 해상도와  화면의 해상도를 매칭시키는 데 사용한다.
					*/
				if(cloud_touch->size() != 0)
				{
					pcl::PointXYZ minPt;
					pcl::getMinMax3D (*cloud_touch, minPt, maxPt);

                                	if (MAX.x < maxPt.x)
                                        	MAX.x = maxPt.x;
                                	if (MAX.y < maxPt.y)
                                       		MAX.y = maxPt.y;
                                	if (MIN.x > z_minPt.x)
                                        	MIN.x = minPt.x;
                                	if (MIN.y > z_minPt.y)
                                        	MIN.y = minPt.y;

                                	std::cout << "MAX: " << "("<< MAX.x << ", "<< MAX.y << ")" <<std::endl;
                                	std::cout << "MIN: " << "("<< MIN.x << ", "<< MIN.y << ")" <<std::endl;
					
					draw_max_min_line(touch_box_max_z);
				}
				// ----------------------------------------------------------Estimate MIN MAX END
			#endif

				if (cloud_filtered->size() == 0 && exe_once == true)	// need for pick_hold mode	// 향후 조건 변경 if(!strcmp(mode, "nothing") && exe_once == true)
				{
					fork_mouse_event(0, 0, (char *)"mouse_up");
					exe_once = false;
				}

				// Handling Touch Box
				// 터치되었을 때 작동
                                if (cloud_touch->size() != 0)
                                {
					
					pcl::PointXYZ touchPt;

                                	int current_min_index = 0;
                                	for(size_t i=0; i < cloud_touch->points.size(); i++)
                                	{
                                        	if(cloud_touch->points[i].z <= cloud_touch->points[current_min_index].z)
                                        	{
                                                	current_min_index = i;
                                        	}

                                	}

                                	touchPt.x = cloud_touch->points[current_min_index].x;
                               		touchPt.y = cloud_touch->points[current_min_index].y;
                                	touchPt.z = cloud_touch->points[current_min_index].z;
                                	viewer->addSphere(touchPt, 0.1, 0.0, 0.0, 1.0, "touchPt");	

                                        // 조건 없이 터치가 되는 자리로 마우스 이동.
					fork_mouse_event(touchPt.x, touchPt.y, (char *)"move");

					if(!strcmp(mode, "click_once"))
						fork_mouse_event(touchPt.x, touchPt.y, (char *)"click");
					if(!strcmp(mode, "click_twice"))
					{
						fork_mouse_event(touchPt.x, touchPt.y, (char *)"click");
						fork_mouse_event(touchPt.x, touchPt.y, (char *)"click");
					}
					// ++++++++++ MODE: pick_hold
					if(!strcmp(mode, "pick_hold"))
					{
						if(exe_once == false)
						{
							fork_mouse_event(touchPt.x, touchPt.y, (char *)"mouse_down");
							exe_once = 1;
						}
					}
					
					draw_tp_box(touch_box_min_z, touch_box_max_z, 0.0, 1.0, 0.0);
                                }else
				{
					if(exe_once == true)
					{
						fork_mouse_event(z_minPt.x, z_minPt.y, (char *)"move");
					}
					draw_tp_box(touch_box_min_z, touch_box_max_z, 1.0, 0.0, 0.0);
				}


				// ++++++++++ MODE: zoom_scroll
				// pressed_finger[0][1]:left_middle_finger	pressed_finger[1][1]:right_middle_finger
				if ((!strcmp(mode, "zoom_scroll")) && cloud_filtered->size() != 0)
				{

					// K-means clustering START
                                	// (Use when the number of hands are known as two...)
					pcl::Kmeans real(static_cast<int> (cloud_filtered->points.size()), 3);
					real.setClusterSize(2);
					for (size_t i = 0; i < cloud_filtered->points.size(); i++)	
					{
						std::vector<float> data(3);
						data[0] = cloud_filtered->points[i].x;
						data[1] = cloud_filtered->points[i].y;
						data[2] = cloud_filtered->points[i].z;
						real.addDataPoint(data);
					}

					real.kMeans();
					pcl::Kmeans::Centroids centroids = real.get_centroids();
					
					std::cout << "===== K-means Cluster Extraction =====" << std::endl;
					pcl::PointXYZ centr1, centr2;
				
					centr1.x = centroids[0][0];
					centr1.y = centroids[0][1];
					centr1.z = centroids[0][2];

					centr2.x = centroids[1][0];
					centr2.y = centroids[1][1];
					centr2.z = centroids[1][2];

					std::cout << "centr1 : ("<< centr1.x << "," << centr1.y << ")"<<std::endl;
					std::cout << "centr2 : ("<< centr2.x << "," << centr2.y << ")"<<std::endl;

					viewer->addSphere(centr1, 0.1, 1.0, 0.0, 0.0, "sphere_0");
					viewer->addSphere(centr2, 0.1, 1.0, 0.0, 0.0, "sphere_1");
					// K-means clustering END
					
					
					// QUEUE가 필요하지 않을 수도 있음. 후에 수정가능.
					// Get distance between centr1 and centr2 and then PUSH it to the queue
					dis.push(xy_distance(centr1, centr2));	//큐에는 거리의 변화량이 아니라 거리가 입력됨.
					
					// Make queue's size 3
					if (dis.size()>3)
					{
						dis.pop();
					}

					std::cout <<"dis queue BACK: " << dis.back() << std::endl;
					std::cout <<"dis queue FRONT: " << dis.front() << std::endl;
					
					// Calculate xy_distance variation
					float dis_variation = dis.back() - dis.front();

					std::cout << "dis_variation: " << dis_variation << std::endl;
					
					// for printing dis_variation
					char text[256];
					sprintf(text, "%f", dis_variation);

					if (dis_variation  > 0.08)	//0.05?? 상황에 따라 조절
					{

						fork_mouse_event(0.0, 0.0, (char *)"scroll_up");
						dis.pop();
						viewer->addText(text, 1280/2, 960/2, 60, 0.0, 0.0, 1.0, "dis_variation");
					}
					else if (dis_variation < -0.08)
					{

						fork_mouse_event(0.0, 0.0, (char *)"scroll_down");
						dis.pop();
						viewer->addText(text, 1280/2, 960/2, 60, 1.0, 0.0, 0.0, "dis_variation");
					}
					else
					{
						viewer->addText(text, 1280/2, 960/2, 60, 0.0, 1.0, 0.0, "dis_variation");
					}
				
				}else
				{
					std::cout << "[CLEAR]: dis queue" << std::endl;
					while(!dis.empty())
					{
						dis.pop();
					}
					std::cout << "queue size after [CLEAR]: " << dis.size() << std::endl;

				}



				// Update cloud on viewer
				viewer->removePointCloud("cloud");
				viewer->addPointCloud(cloud_filtered,"cloud");
			}
		}

		interface->stop ();
	}

	pcl::visualization::PCLVisualizer::Ptr viewer;
};


int main (int argc, char** argv)
{
	GestureHandler gh;
	gh.viewer_set();
	gh.run ();
	return 0;
}

