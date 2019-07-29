#ifndef __MAIN_HEADER__
#define __MAIN_HEADER__

#include <iostream>

#include <queue>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>	//	for Euclidian Cluster Extraction

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
		viewer->setCameraPosition(0,0,-3.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0);
	}

	void run ()
	{
		boost::mutex mutex;

		pcl::PointCloud<PointT>::ConstPtr cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

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

		/*
		// Create the KdTree object for the search method of the extraction
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		// for Euclidian Cluster
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointT> ec;
  		ec.setClusterTolerance (0.1); // 0.01 -> 1cm
  		ec.setMinClusterSize (300);
  		ec.setMaxClusterSize (20000); // 10000
		*/

		//int rm_cnt=0;
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
				pass.setFilterLimits(0.0, 11.0);//9.0
				pass.filter(*cloud_filtered);
				// PassThrough Filtering END

				

				// Remove shapes to update shapes START
				std::cout << "===== REMOVE SHAPE =====" << std::endl;
				for(;rm_cnt >= 0; rm_cnt--)
				{
					std::stringstream remove_id;
					remove_id << "sphere_"<< rm_cnt;
					
					std::cout << "removeSphere id : " << remove_id.str() << std::endl;

					viewer->removeShape(remove_id.str());

					viewer->removeShape("minPt_z");
				}
				// Reomve shapes END


				/*
				// To recognize the number of hands
                                // Euclidean Cluster Extraction START
                                std::vector<pcl::PointIndices> cluster_indices;
                            	// ec : Euclidian Cluster Object || tree : KdTree Object
                                tree->setInputCloud (cloud_filtered);
                                ec.setSearchMethod (tree);
                                ec.setInputCloud (cloud_filtered);
                                ec.extract (cluster_indices);
				
					
				std::cout << "===== NEW CLUSTER =====" << std::endl;
				int j = 0;
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
				{
					pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
					for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
						cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
					cloud_cluster->width = cloud_cluster->points.size ();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;

    					std::cout << "cloud_cluster["<<j<<"] representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    					
					std::stringstream sphere_id;
					sphere_id << "sphere_" << j;

					viewer->addSphere(cloud_cluster->points[0], 0.1, 1.0, 0.0, 0.0, sphere_id.str());

    					j++;
  				}
				rm_cnt = j-1;//int 
				// Euclidean Cluster Extraction END
				*/
				
				// Get_max & min_coordinates
                                pcl::PointXYZ minPt, maxPt;
                                pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
                                std::cout << "Min z: " << minPt.z << std::endl;
                                minPt.x = 0.0;
                                minPt.y = 0.0;

				// MOUSE EVENT TEST (WARN : JUST TEST! THIS GONNA BE MODIFIED LATER!)
                                if (minPt.z<5.0)
                                {
                                        fork_mouse_event((char *)"click");
                                }


				if (button_pressed == 2)
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
					std::cout << "centroid count : " << centroids.size() << std::endl;
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
					rm_cnt=1;	
					// K-means clustering END
				
					// Get distance between centr1 and centr2
					if (cloud_filtered->size()!=0)
					{
						dis.push(xy_distance(centr1, centr2));
						std::cout << "xy_dis: " << dis.back() << std::endl;
					}
					if (dis.size()>10)	// 수정할 부분.. 거리의 변화에 따라 바꿔야 할 듯 
					// (둘 사이의 거리가 거의 변화하지 않을 경우를 생각
					// -> 만약 큐에 일정 개수 이상의 데이터가 쌓일 경우 앞부분부터 pop을 한다.)
					// (거리가 일정 거리 이상 변화했을 경우를 생각
					// -> 마우스를 컨트롤하고 while문을 통해 queue를 비운다.)
					{
						dis.pop();
					}
					std::cout <<"dis queue FRONT: " << dis.front() << std::endl;
				
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
				viewer->addSphere(minPt, 0.1, 0.0, 0.0, 1.0, "minPt_z");

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

