#include <iostream>

#include <queue>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>			// for using distance();
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>				// for K-means Cluster Extraction

#include <boost/thread.hpp>

#include "proj/etc.hpp"

//-------------------<사용자 설정 값>-----------------------
#define TOUCH_Z_MAX	10.2				// TOUCH_Z_MAX를 변경할 경우, Screen_data 구조체의 값들을 새롭게 측정한 실측값으로 변경.
//#define TEST_CUBE
//#define UNITY_MODE					// Unity3D 연동 모드.

#ifdef TEST_CUBE
#define TEST_CUBE_LEN	1.0
#endif

//#define Estimate_MIN_MAX				// TOUCH_Z_MAX를 변경할 경우, 주석을 해제하여 변경할 영역에서의 MIN, MAX 측정.
//----------------------------------------------------------

using namespace std;
typedef pcl::PointXYZ PointT;

int pressed_finger[2][2] = {{0,0}, {0,0}};
struct Gyro gyro;

class GestureHandler
{
private:
	/*
	 * Screen_data: 터치영역과 출력화면의 정보를 담고 있는 구조체
	 * 파라미터 입력용 포인터
	 */
	struct Screen_data screen_data;
	struct Screen_data *sd = &screen_data;

	/*
	 *  pressed_finger[0][0]: 오른쪽 손 검지
	 *  pressed_finger[0][1]: 오른쪽 손 중지
	 *  pressed_finger[1][0]: 왼쪽 손 검지
	 *  pressed_finger[1][1]: 왼쪽 손 중지
	 */

        int (*pressed_finger_Ptr)[2];			// pressed_finger를 파라미터로 전달하는 것을 목적으로 하는 2차원 포인터 변수.

	/*
	 * mode "nothing": 터치를 했을 경우 마우스 이동만 한다.
	 * mode "zoom_scroll": (양손 사이의 거리를 늘이거나 줄이면) 줌 인 줌 아웃 기능을 한다.
	 * mode "pick_hold": (이 모드에 진입한 상태로 터치를 하면) 터치한 자리에 mousedown 클릭을 한다.
	 * mode "": 
	 */
        char mode[32];
	char mode_past[32];

	/*
	 * keyboardEventOccured()에 파라미터로 입력된다. 
	 * 마지막으로 입력된 키에 대한 정보가 저장된다.
	 */
	char key_id[32];

	/*
	 * mode "zoom_scroll"와 같은 모드의 유지, 해제에 이용된다. 
	 * 어떠한 모드를 유지시킬 경우에는 true, 유지시키 않을 경우에는 false로 전환한다.
	 */
        int exe_once=0;		// bool형으로 고칠 것

	/*
	 * 프레임마다 클릭을 반복하지 않고, 
	 * 터치 박스 영역에 입력이 들어왔을 경우 
	 * 한번만 클릭을 수행하도록 하기 위해 이용된다.
	 */
	bool last_frame_touched = false;

        queue<float> dis;       			// Save distances between centr1 and centr2.

	queue<PointT> point_q;

	queue<float> x_q;
	queue<float> y_q;

#ifdef TEST_CUBE
	pcl::PointXYZ shape_touchedPt;
	int coll_detect(pcl::PointCloud<PointT>::Ptr, PointT, float, char *);
	pcl::PointXYZ cube_centr;
	pcl::PointXYZ cube_past;
	void drawCustomCube(PointT, float, double, double, double);
#endif

	void draw_max_min_line(float);
	void draw_tp_box(float, float, double, double, double);

public:	
//////////////////////////////////// Public Variables

	void *data_struct_ptr;

//////////////////////////////////// Public Functions

	/* Viewer 설정 초기화 */
	void viewer_set()
	{
		viewer->setFullScreen(false);
		viewer->setSize(1280, 960);
		//viewer->addCoordinateSystem(1.0);
		viewer->setCameraPosition(sd->Cloud_x_center, sd->Cloud_y_center, 25.0, 0.0,0.0,-1.0, 0.0,0.0,0.0, 0);

	}


	/* Constructor(생성자) */
	GestureHandler () : 
		viewer (new pcl::visualization::PCLVisualizer ("Gesture Handler 3D Viewer"))	// 동적 메모리 할당
	{
		
		data_struct_ptr = init_miniterm();

		viewer_set();

		viewer->registerMouseCallback(mouseEventOccurred, (void *)&viewer);
		viewer->registerKeyboardCallback(keyboardEventOccurred, (void *)&key_id);

#ifdef TEST_CUBE	
		strcpy(mode, "cube_touch");// TEST_CUBE
		cube_centr.x = sd->Cloud_x_center;
		cube_centr.y = sd->Cloud_y_center;
		cube_centr.z = TOUCH_Z_MAX+1.0;
#endif//TEST_CUBE
#ifdef UNITY_MODE
		fork_unity();
#endif//UNITY_MODE

		pressed_finger_Ptr = pressed_finger;
		detect_mode(mode, pressed_finger_Ptr);	// 함수 2번째 파라미터에 2차원 포인터 변수를 넘겨준다.
	
	}

	/* GestureHandler Main func */
	void run ()
	{
		boost::mutex mutex;

		//-------------------------------------
		float touch_box_min_z=TOUCH_Z_MAX-0.2;
		float touch_box_max_z=TOUCH_Z_MAX;
		float cloud_filtered_max_z = TOUCH_Z_MAX+4.8;
		//-------------------------------------

#ifdef ESTIMATE_MIN_MAX
		pcl::PointXYZ MIN; pcl::PointXYZ MAX;	// Use for estimating MAX values or MIN values of the coordinates.
#endif//ESTIMATE_MIN_MAX

		/*
		 * cloud에 대해 'z'기준 PassThrough Filtering을 거친 것이 cloud_filtered 이다.
		 * cloud_filtered에 대해 'z'기준 PassThrough Filtering을 거친 것이 cloud_touch 이다.
		 */
		pcl::PointCloud<PointT>::ConstPtr cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_touch (new pcl::PointCloud<PointT>);


		std::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f =
		[&cloud, &mutex](const pcl::PointCloud<PointT>::ConstPtr &ptr )
		{
			boost::mutex::scoped_lock lock( mutex );	// scoped_lock은 scope를 벗어나면 자동으로 unlock을 해준다.
			cloud = ptr->makeShared();
		};

		/* Initialize Grabber Ptr ("interface") */
		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber(
                "", pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz,
                pcl::io::OpenNI2Grabber::OpenNI_QVGA_60Hz);		// QVGA_60Hz Recommended (VGA_30Hz or QVGA_60Hz)
		
		interface->registerCallback (f);

		interface->start ();

		// Create the PassThrough Filtering object
		pcl::PassThrough<PointT> pass;

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
			boost::mutex::scoped_try_lock lock( mutex );	// try_lock() 비동기식에 이용가능 자신의 lock이면 true를 반환, 아니면 false 반환
			if(lock.owns_lock())
			{

				std::cout << "-----------------------------------------------------------" << std::endl;
#ifndef TEST_CUBE

				//if(read_num == 1)
				//{
					// pressed_finger의 정보를 바탕으로 mode 지정.
					detect_mode(mode, pressed_finger);
				//}
#endif//TEST_CUBE

				printf("issame : %d", issame);
				std::cout << "[detect_mode]: "<< mode << std::endl;

				// Remove shapes to update shapes START
				std::cout << "[REMOVE SHAPE]: ALL" << std::endl;
				viewer->removeAllShapes();
				// Reomve shapes END


				// PassThrough Filtering START
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(touch_box_min_z, cloud_filtered_max_z); //QVGA 기준 || 0.0~1.0?? in VGA
				pass.filter(*cloud_filtered);

				pass.setInputCloud(cloud_filtered);
				pass.setFilterLimits(touch_box_min_z, touch_box_max_z);
				pass.filter(*cloud_touch);	// This cloud_touch will be in Touch Box Field
				// PassThrough Filtering END
				
				
				// Get_max_and_min_coordinates
				// Get Z Minimum Point
				/*
                                 * z_minPt는 z값이 가장 작은 점의 xyz좌표를 담는 구조체
                                 * maxPt는 가장 큰 x, 가장 큰 y, 가장 큰 z를 각각 담는 구조체
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
				// Estimate MIN MAX START
                                /* 
				 * 화면 비율과 좌표를 맞추기 위해 cloud_touch의 MIN, MAX를 측정한다.
                                 * PointCloud 상의 최대 x, y 좌표를 측정하여 TouchBox의 해상도와  화면의 해상도를 매칭시키는 데 사용한다.
				 */
				if(cloud_touch->size() != 0)
				{
					pcl::PointXYZ minPt;
					pcl::getMinMax3D (*cloud_touch, minPt, maxPt);

                                	if (MAX.x < maxPt.x)
                                        	MAX.x = maxPt.x;
                                	if (MAX.y < maxPt.y)
                                       		MAX.y = maxPt.y;
                                	if (MIN.x > minPt.x)
                                        	MIN.x = minPt.x;
                                	if (MIN.y > minPt.y)
                                        	MIN.y = minPt.y;

                                	std::cout << "MAX: " << "("<< MAX.x << ", "<< MAX.y << ")" <<std::endl;
                                	std::cout << "MIN: " << "("<< MIN.x << ", "<< MIN.y << ")" <<std::endl;
					
					draw_max_min_line(touch_box_max_z);
				}
				// Estimate MIN MAX END
#endif//ESTIMATE_MIN_MAX

				// If the mode is changed...
				/*
				 * 이전 프레임의 모드인 mode_past와 mode가 다를 경우(즉, 모드가 바뀌었을 경우)
				 * 혹은 점 데이터가 없는 경우(즉, 손이 최대 인식 영역을 벗어났을 경우)
				 * 모드를 위해 사용되었던 변수들을 기본 상태로 초기화해준다.
				 */
				if (strcmp(mode_past, mode) || cloud_filtered->size() == 0)				// This mean that the mode is changed.
				{
					if(exe_once == true)
					{
						fork_xdotool_event(sd, 0, 0, (char *)"mouse_up");		// mouse_down을 해제시킨다.
						exe_once = false;					// pick_hold 모드 유지가 해제된다.
					}

					// "zoom" mode에 이용되던 큐 비우기
					std::cout << "[CLEAR]: dis queue" << std::endl;
					while(!dis.empty())
					{
						dis.pop();
					}
					std::cout << "queue size after [CLEAR]: " << dis.size() << std::endl;
				}


#ifdef TEST_CUBE
				if(!strcmp(key_id, "key_t"))
				{
					strcpy(mode, "cube_pick");
				}
				else if(!strcmp(key_id, "key_y"))
				{
					strcpy(mode, "cube_touch");
				}
				
				if(coll_detect(cloud_filtered, cube_centr, TEST_CUBE_LEN, (char *)"CustomCube") == true)	//충돌했을 경우
				{
					if(!strcmp(mode, "cube_touch"))
						drawCustomCube(cube_centr, TEST_CUBE_LEN, 0.0, 0.0, 1.0);	
					else if(!strcmp(mode, "cube_pick"))
					{
						cube_centr.x = z_minPt.x;
						cube_centr.y = z_minPt.y;
						cube_centr.z = z_minPt.z;
						/*
						if(cube_centr.x > cube_past.x + 0.05)
						{
							fork_xdotool_event(sd, 0,0, (char *)"key_Right");
						}
						else if(cube_centr.x < cube_past.x - 0.05)
						{
							fork_xdotool_event(sd, 0,0, (char *)"key_Left");
						}
						*/
						if(cube_centr.z > cube_past.z + 0.05)
						{
							fork_xdotool_event(sd, 0,0, (char *)"key_Down");
						}
						else if(cube_centr.z < cube_past.z - 0.05)
						{
							fork_xdotool_event(sd, 0,0, (char *)"key_Up");
						}
						cube_past.x = cube_centr.x;
						cube_past.y = cube_centr.y;
						cube_past.z = cube_centr.z;
						
						drawCustomCube(cube_centr, TEST_CUBE_LEN, 0.0, 0.0, 1.0);
					}
					
				}
				else	//충돌하지 않았을 경우
				{
					if(!strcmp(mode, "cube_touch"))
						drawCustomCube(cube_centr, TEST_CUBE_LEN, 1.0, 0.0, 0.0);	
					else if(!strcmp(mode, "cube_pick"))
						drawCustomCube(cube_centr, TEST_CUBE_LEN, 0.0, 1.0, 0.0);
				}

#endif//TEST_CUBE



// -----------------------------Handling Touch Box START
				// 터치되었을 때 작동
                                if (cloud_touch->size() != 0)
                                {
					
					pcl::PointXYZ touchPt;

					/* cloud_touch 터치 영역에서 Z좌표가 가장 작은 점을 지정 */
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

                                        // 조건 없이 터치가 되는 자리(touchPt)로 마우스 이동.
					fork_xdotool_event(sd, touchPt.x, touchPt.y, (char *)"move");
#ifdef UNITY_MODE	////////////////////////////////////////////////////////////////////////////////////
					if (!last_frame_touched)
					{
						fork_xdotool_event(sd, touchPt.x, touchPt.y, (char *)"click");
					}
					last_frame_touched = true;
#endif//UNIIY_MODE

					if(!strcmp(mode, "click_once"))
						fork_xdotool_event(sd, touchPt.x, touchPt.y, (char *)"click");
					if(!strcmp(mode, "click_twice"))
					{
						fork_xdotool_event(sd, touchPt.x, touchPt.y, (char *)"click");
						fork_xdotool_event(sd, touchPt.x, touchPt.y, (char *)"click");
					}
/*
#ifndef UNITY_MODE
#endif//UNITY_MODE
*/
					
					draw_tp_box(touch_box_min_z, touch_box_max_z, 0.0, 1.0, 0.0);
                                }

				else	// cloud_touch에 속한 점 데이터가 없을 경우 (터치하지 않았을 경우)
				{

					// ++++++++++ MODE: "pick_hold"
					/* 
					 * 하나의 중지를 활성화시킨 상태로 터치영역 안에 들어갔을 때부터 중지를 계속 활성되어 있는 동안 mouse_down을 수행하고 싶다면 다음 조건문을 if(cloud_touch->size()!=0)문 안에 넣는다.
					 * 터치 여부와 관계없이 하나의 중지를 활성화시킨 상태일 때 mouse_down을 수행하고 싶다면 다음 조건문을 if(cloud_touch->size()!=0)밖에 둔다.
					 */
					if(!strcmp(mode, "pick_hold") && strcmp(mode_past, "pick_hold"))
					{
							fork_xdotool_event(sd, z_minPt.x, z_minPt.y, (char *)"move");
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"mouse_down");

					}
					else if(!strcmp(mode, "pick_hold") && !strcmp(mode_past, "pick_hold"))
					{
							fork_xdotool_event(sd, z_minPt.x, z_minPt.y, (char *)"move");
					}
					else if(strcmp(mode, "pick_hold") && !strcmp(mode_past, "pick_hold"))
					{
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"mouse_up");
					}

					last_frame_touched = false;
					/*
					if(exe_once == true)	// 모드 유지가 활성화되어 있는 상태일 경우
					{

						if(!strcmp(mode, "pick_hold"))	// "pick_hold" mode가 터치영역 밖에서 유지되어 있는 경우
						{
							fork_xdotool_event(sd, z_minPt.x, z_minPt.y, (char *)"move");
						}
						//else if(!strcpy(mode, ""))
					}
					*/
					draw_tp_box(touch_box_min_z, touch_box_max_z, 1.0, 0.0, 0.0);
				}

// -----------------------------Handling Touch Box END



// -----------------------------Handling NON Touch Box START

				if (cloud_filtered->size() != 0 && cloud_touch->size() == 0)
				{

#ifdef UNITY_MODE	///////////////////////////////////////////////////////////////////////////////////
					// ++++++++++ MODE: "pick_hold" (in UNITY_MODE)
					if (!strcmp(mode, "pick_hold"))
					{
						
						float x_variation=0.0;
						float y_variation=0.0;
						float z_variation=0.0;						

						dis.push(z_minPt.z);
						x_q.push(z_minPt.x);
						y_q.push(z_minPt.y);
						

						/*
						float euclidian_dist;
						
						point_q.push(z_minPt);
						if(point_q.size()>2)
						{
							euclidian_dist = 
								  pow(point_q.back().x-point_q.front().x, 2)
								+ pow(point_q.back().y-point_q.front().y, 2)
								+ pow(point_q.back().z-point_q.front().z, 2);

							dis.push(euclidian_dist);
						}

						if(dis.size()>2)
						{
							dis.pop();
							std::cout << "Z variation: " << point_q.back().x - point_q.front().x << std::endl;
						}
						*/
						
						if(dis.size()>2)
						{
							dis.pop();
							z_variation = dis.back() - dis.front();
							std::cout << "Z 변화량 : "<< z_variation << std::endl;
						}
						if(x_q.size()>2)
						{
							x_q.pop();
							x_variation = x_q.back() - x_q.front();
						}
						if(y_q.size()>2)
						{
							y_q.pop();
							y_variation = y_q.back() - y_q.front();
						}
#define TEL
						if(z_variation > 0.02)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_Y");
						}
						else if(z_variation < -0.02)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_T");
						}
//--------------------------------------------------
						if(x_variation > 0.01)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_Right");
						}
						else if(x_variation < -0.01)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_Left");
						}

						if(y_variation > 0.01)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_Up");//
						}
						else if(y_variation < -0.01)
						{
							//for(int j=0; j<3; j++)
								fork_xdotool_event(sd, 0,0, (char *)"key_Down");//
						}
//-------------------------------------------------
						

					}

#endif//UNITY_MODE
					// ++++++++++ MODE: "zoom_scroll"
					if (!strcmp(mode, "zoom_scroll"))
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
						
						// Calculate xy_distance variation.
						float dis_variation = dis.back() - dis.front();
						std::cout << "dis_variation: " << dis_variation << std::endl;
						
						// for printing dis_variation.
						char text[256];
						sprintf(text, "%f", dis_variation);
	
						if (dis_variation  > 0.08)
						{
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"scroll_up");
							//input_event();
							dis.pop();
							viewer->addText(text, 1280/2, 960/2, 60, 0.0, 0.0, 1.0, "dis_variation");	// Windows size : 1280 X 960
						}
						else if (dis_variation < -0.08)
						{
	
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"scroll_down");
							dis.pop();
							viewer->addText(text, 1280/2, 960/2, 60, 1.0, 0.0, 0.0, "dis_variation");
						}
						else
						{
							viewer->addText(text, 1280/2, 960/2, 60, 0.0, 1.0, 0.0, "dis_variation");
						}
				
					}

					

				}
// -----------------------------Handling NON Touch Box END

#ifdef UNITY_MODE
				if(!strcmp(mode, "unity_shoot"))
					fork_xdotool_event(sd, 0.0, 0.0, (char *)"key_Space");
#endif		

				printf("GYRO>> x:%f y:%f z:%f \n", gyro.x, gyro.y, gyro.z);
				if(!strcmp(mode, "spin_hold"))
				{
					if(strcmp(mode_past, mode))
						y_q.push(gyro.y);

					while(y_q.size() > 2)
						y_q.pop();

					y_q.push(gyro.x);

					if(y_q.size() == 2)
					{
						if(y_q.front()-y_q.back() >= 0.03)		//왼쪽으로 기울어졌을 경우
						{
						
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"keydown_Alt");
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"key_Right");	//or left
						}
						else if(y_q.front()-y_q.back() <= -0.03)	//오른쪽으로 기울어졌을 경우
						{
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"keydown_Alt");
							fork_xdotool_event(sd, 0.0, 0.0, (char *)"key_Left");	//or left
						}
						
					}
					
				}
				else
				{
					while(!y_q.empty())
						y_q.pop();

					if(!strcmp(mode_past, "spin_hold"))
						fork_xdotool_event(sd, 0.0, 0.0, (char *)"keyup_Alt");
				}


				// 다음 프레임에서 이전의 프레임의 모드가 될 mode_past의 값 저장.
				strcpy(mode_past, mode);

				// Update cloud on viewer.
				viewer->removePointCloud("cloud");
				viewer->addPointCloud(cloud_filtered,"cloud");

				printf("\e[2J\e[H");
			}
		}

		interface->stop ();
	}

	pcl::visualization::PCLVisualizer::Ptr viewer;
};


#ifdef TEST_CUBE
/*
 * First Parameter:  충돌을 검사할 PointCloud포인터
 * Second Parameter: 충돌을 검사할 Shape의 위치(Position)
 * Third Parameter:  충돌을 검사할 Shape의 길이 정보(CustomCube: 변의 길이, CustomSphere: 반지름의 길이)
 * Fourth Parameter: 충돌을 검사할 Shape의 id
 * 충돌이 검출되면 1를 반환하고 그렇지 않으면 0을 반환.
 */
int GestureHandler::coll_detect(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointXYZ pos, float len, char *id)
{
	if(!strcmp(id, "CustomCube"))
	{
		for(int i=0; i < cloud->size(); i++)
		{
			if((cloud->points[i].x >= (pos.x-len/2)) && (cloud->points[i].y >= (pos.y-len/2)) && (cloud->points[i].z >= (pos.z-len/2)))
				if((cloud->points[i].x <= (pos.x+len/2)) && (cloud->points[i].y <= (pos.y+len/2)) && (cloud->points[i].z <= (pos.z+len/2)))
				{
					std::cout << "[COLLISION_DETECTED]: (id)CustomCube"<<std::endl;
					this->shape_touchedPt = cloud->points[i];
					return 1;
				}
		}
		return 0;
	}
	if(!strcmp(id, "CustomSphere"))
	{
		float dist;
		for(int i=0; i < cloud->size(); i++)
		{
			dist = pcl::geometry::distance(cloud->points[i], pos);
			if(dist <= len)
			{
				std::cout << "[COLLISION_DETECTED]: (id)CustomSphere"<<std::endl;
				return 1;
			}
		}
		return 0;
	}
}

void GestureHandler::drawCustomCube(pcl::PointXYZ pos, float len, double r, double g, double b)
{
	viewer->addCube(pos.x-len/2, pos.x+len/2, pos.y-len/2, pos.y+len/2, pos.z-len/2, pos.z+len/2, r, g, b, "CustomCube");
}
#endif


/* 터치 영역의 박스 정중앙을 기준으로 십자선을 그린다. */
void GestureHandler::draw_max_min_line(float z_max)
{
	pcl::PointXYZ a;
	pcl::PointXYZ b;

	a.x = sd->MIN_X;
	a.y = (sd->MIN_Y+sd->MAX_Y)/2;
	a.z = z_max;
	
	b.x = sd->MAX_X;
	b.y = (sd->MIN_Y+sd->MAX_Y)/2;
	b.z = z_max;
	
	viewer->addLine(a, b, 0, 0, 1.0, "Cloud_width");
	
	a.x = (sd->MIN_X+sd->MAX_X)/2;
	a.y = sd->MIN_Y;
	a.z = z_max;
	
	b.x = (sd->MIN_X+sd->MAX_X)/2;
	b.y = sd->MAX_Y;
	b.z = z_max;
	
	viewer->addLine(a, b, 0, 0, 1.0, "Cloud_height");
}

/*
 * Draw transparent box
 * 투명한 터치 영역 박스를 시각화.
 */
void GestureHandler::draw_tp_box(float z_min, float z_max, double _r,  double _g, double _b)
{
	pcl::PointXYZ a;
	pcl::PointXYZ b;

	a.x = sd->MIN_X;
	a.y = sd->MAX_Y;
	a.z = z_max;
	b.x = sd->MAX_X;
	b.y = sd->MAX_Y;
	b.z = z_max;
	viewer->addLine(a, b, _r,_g,_b, "line_x_1");

	a.x = sd->MIN_X;
	a.y = sd->MIN_Y;
	a.z = z_max;
	b.x = sd->MAX_X;
	b.y = sd->MIN_Y;
	b.z = z_max;
	viewer->addLine(a, b, _r,_g,_b, "line_x_2");

	a.x = sd->MIN_X;
	a.y = sd->MIN_Y;
	a.z = z_min;
	b.x = sd->MAX_X;
	b.y = sd->MIN_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_x_3");

	a.x = sd->MIN_X;
	a.y = sd->MAX_Y;
	a.z = z_min;
	b.x = sd->MAX_X;
	b.y = sd->MAX_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_x_4");

	a.x = sd->MIN_X;
	a.y = sd->MAX_Y;
	a.z = z_max;
	b.x = sd->MIN_X;
	b.y = sd->MIN_Y;
	b.z = z_max;
	viewer->addLine(a, b, _r,_g,_b, "line_y_1");

	a.x = sd->MAX_X;
	a.y = sd->MIN_Y;
	a.z = z_max;
	b.x = sd->MAX_X;
	b.y = sd->MIN_Y;
	b.z = z_max;
	viewer->addLine(a, b, _r,_g,_b, "line_y_2");

	a.x = sd->MAX_X;
	a.y = sd->MAX_Y;
	a.z = z_min;
	b.x = sd->MAX_X;
	b.y = sd->MIN_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_y_3");

	a.x = sd->MIN_X;
	a.y = sd->MAX_Y;
	a.z = z_min;
	b.x = sd->MIN_X;
	b.y = sd->MIN_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_y_4");

	a.x = sd->MIN_X;
	a.y = sd->MAX_Y;
	a.z = z_min;
	b.x = sd->MAX_X;
	b.y = sd->MIN_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_d_min1");

	a.x = sd->MAX_X;
	a.y = sd->MAX_Y;
	a.z = z_min;
	b.x = sd->MIN_X;
	b.y = sd->MIN_Y;
	b.z = z_min;
	viewer->addLine(a, b, _r,_g,_b, "line_d_min2");

}



int main (int argc, char** argv)
{
	GestureHandler gh;
	gh.run ();

	free(gh.data_struct_ptr);

	printf("---------------------PROGRAM EXIT-----------------------\n");
	exit(0);
}

