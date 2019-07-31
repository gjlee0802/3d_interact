#ifndef __MAIN_HEADER__
#define __MAIN_HEADER__

#include <iostream>

#include <queue>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>	//	for K-means Cluster Extraction

#include <boost/thread.hpp>

#include "proj/etc.hpp"

#endif

typedef pcl::PointXYZ PointT;
using namespace std;


class HandViewer
{
	public:
		HandViewer () : viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")) {}
		int rm_cnt=0;
		
		int button_pressed=2;	// This should be changed simultaneously by arduino's informations
		
		queue<float> dis;	// Save distances between centr1 and centr2 in this vector container

	void viewer_set()
	{
		viewer->setFullScreen(false);
		viewer->setSize(1280, 960);
		//viewer->addCoordinateSystem(1.0);
		viewer->setCameraPosition(0,0,-6.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0);
	}

	void draw_tp_box(float z_min, float z_max, double _r,  double _g, double _b)
	{
		pcl::PointXYZ a;
		pcl::PointXYZ b;

		float width = 8.0;
		float height = 6.0;

		a.x = width/2*(-1.0);
		a.y = height/2;
		a.z = z_max;
		b.x = width/2;
		b.y = height/2;
		b.z = z_max;
		viewer->addLine(a, b, _r,_g,_b, "line_x_1");

		a.x = width/2*(-1.0);
                a.y = height/2*(-1.0);
                a.z = z_max;
                b.x = width/2;
                b.y = height/2*(-1.0);
                b.z = z_max;
		viewer->addLine(a, b, _r,_g,_b, "line_x_2");

		a.x = width/2*(-1.0);
                a.y = height/2*(-1.0);
                a.z = z_min;
                b.x = width/2;
                b.y = height/2*(-1.0);
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_x_3");

		a.x = width/2*(-1.0);
                a.y = height/2;
                a.z = z_min;
                b.x = width/2;
                b.y = height/2;
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_x_4");

		a.x = width/2*(-1.0);
                a.y = height/2;
                a.z = z_max;
                b.x = width/2*(-1.0);
                b.y = height/2*(-1.0);
                b.z = z_max;
                viewer->addLine(a, b, _r,_g,_b, "line_y_1");

		a.x = width/2;
                a.y = height/2;
                a.z = z_max;
                b.x = width/2;
                b.y = height/2*(-1.0);
                b.z = z_max;
                viewer->addLine(a, b, _r,_g,_b, "line_y_2");

		a.x = width/2;
                a.y = height/2;
                a.z = z_min;
                b.x = width/2;
                b.y = height/2*(-1.0);
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_y_3");

		a.x = width/2*(-1.0);
                a.y = height/2;
                a.z = z_min;
                b.x = width/2*(-1.0);
                b.y = height/2*(-1.0);
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_y_4");

		a.x = width/2*(-1.0);
                a.y = height/2;
                a.z = z_min;
                b.x = width/2;
                b.y = height/2*(-1.0);
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_d_min1");

		a.x = width/2;
                a.y = height/2;
                a.z = z_min;
                b.x = width/2*(-1.0);
                b.y = height/2*(-1.0);
                b.z = z_min;
                viewer->addLine(a, b, _r,_g,_b, "line_d_min2");

	}

	void run ()
	{
		boost::mutex mutex;

		float touch_box_min_z=6.2;
		float touch_box_max_z=6.4;

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

				// Print cloud->size()
				std::cout << "cloud size : " << cloud->size() << std::endl;

				// PassThrough Filtering START
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(0.0, 11.0); //0.0~11.0 in QVGA || 0.0~1.0 in VGA
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
				int current_min_index = 0;
                                pcl::PointXYZ z_min_Pt;
				pcl::PointXYZ minPt, maxPt;

                                pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
                                std::cout << "Min z: " << minPt.z << std::endl;

				for(size_t i=0; i < cloud_filtered->points.size(); i++)
				{
					if(cloud_filtered->points[i].z == minPt.z)
					{
						minPt.x = cloud_filtered->points[i].x;
						minPt.y = cloud_filtered->points[i].y;
						break;
					}
				}
				std::cout << "minPt: ("<< minPt.x <<", "<< minPt.y <<")"<<std::endl;

				std::stringstream text;
				text << "("<<minPt.x<<", "<<minPt.y<<")";
				//viewer -> addText(text.str(), 10, 10, "text_minPt");
				
				// Handling Touch Box
                                if (cloud_touch->size() != 0)
                                {
					
					pcl::PointXYZ touchPt;

                                	current_min_index = 0;
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

                                        fork_mouse_event(touchPt.x, touchPt.y, (char *)"move");
					//fork_mouse_event(touchPt.x, touchPt.y, (char *)"click");

					draw_tp_box(touch_box_min_z, touch_box_max_z, 0.0, 1.0, 0.0);
                                }else
				{
					draw_tp_box(touch_box_min_z, touch_box_max_z, 1.0, 0.0, 0.0);
				}


				// Two hand gesture handler
				if (button_pressed == 2 && cloud_filtered->size() != 0)
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
				
					// Get distance between centr1 and centr2
					dis.push(xy_distance(centr1, centr2));
					std::cout << "xy_dis: " << dis.back() << std::endl;
					
					if (dis.size()>3)	// 수정할 부분.. 거리의 변화에 따라 바꿔야 할 듯 
					// (둘 사이의 거리가 거의 변화하지 않을 경우를 생각
					// -> 만약 큐에 일정 개수 이상의 데이터가 쌓일 경우 앞부분부터 pop을 한다.)
					// (거리가 일정 거리 이상 변화했을 경우를 생각
					// -> 마우스를 컨트롤하고 while문을 통해 queue를 비운다.)
					{
						dis.pop();
					}
					std::cout <<"dis queue FRONT: " << dis.front() << std::endl;
					
					pcl::PointXYZ scroll;

					scroll.x = 0.0; scroll.y = 0.0; scroll.z = 5.0;
					
					float dis_variation = dis.back() - dis.front();
					
					float temp = dis.back();

					std::cout << "dis_variation: " << dis_variation << std::endl;
					
					char text[256];
					sprintf(text, "%f", dis_variation);

					if (dis_variation  > 0.08)	//0.05?? 상황에 따라 조절
					{

						fork_mouse_event(0.0, 0.0, (char *)"scroll_up");
						/*
						std::cout << "[CLEAR]: dis queue" << std::endl;
                                        	while(!dis.empty())
                                        	{
                                                	dis.pop();
                                        	}
						dis.push(temp);
                                        	std::cout << "queue size after [CLEAR]: " << dis.size() << std::endl;*/
						dis.pop();
						viewer->addText(text, 1280/2, 960/2, 60, 0.0, 0.0, 1.0, "dis_variation");
					}
					else if (dis_variation < -0.08)
					{

						fork_mouse_event(0.0, 0.0, (char *)"scroll_down");
						/*
						std::cout << "[CLEAR]: dis queue" << std::endl;
                                        	while(!dis.empty())
                                        	{
                                                	dis.pop();
                                        	}
						dis.push(temp);
                                        	std::cout << "queue size after [CLEAR]: " << dis.size() << std::endl;*/
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

				// Update viewer
				
				//viewer->addSphere(minPt, 0.1, 0.0, 0.0, 1.0, "minPt_z");

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
	HandViewer v;
	v.viewer_set();
	v.run ();
	return 0;
}

