/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Imu.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

#include "tf/transform_broadcaster.h"

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;
ros::Publisher imu_m;
ros::Subscriber imu_sub;
using namespace cv;
sensor_msgs::Imu imu_p;

//std::string se;
bool flag=true;






#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
e_vbus_index	CAMERA_ID = e_vbus1;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat	        g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat	        g_depth(HEIGHT,WIDTH,CV_16SC1);

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();


    /* image data
    if (e_image == data_type && NULL != content)
    {        
        image_data data;
        memcpy((char*)&data, content, sizeof(data));

        memcpy(g_greyscale_image_left.data, data.m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
        memcpy(g_greyscale_image_right.data, data.m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
        memcpy(g_depth.data, data.m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);

        Mat depth8(HEIGHT, WIDTH, CV_8UC1);
        g_depth.convertTo(depth8, CV_8UC1);
        imshow("left",  g_greyscale_image_left);
	imshow("right", g_greyscale_image_right);
        imshow("depth", depth8);
        key = waitKey(1);

        //publish depth image
        cv_bridge::CvImage depth_16;
        g_depth.copyTo(depth_16.image);
        depth_16.header.frame_id  = "guidance";
        depth_16.header.stamp	  = ros::Time::now();
        depth_16.encoding	  = sensor_msgs::image_encodings::MONO16;
        depth_image_pub.publish(depth_16.toImageMsg());

        // publish left greyscale image
        cv_bridge::CvImage left_8;
        g_greyscale_image_left.copyTo(left_8.image);
        left_8.header.frame_id  = "guidance";
        left_8.header.stamp	= ros::Time::now();
        left_8.encoding		= sensor_msgs::image_encodings::MONO8;
        left_image_pub.publish(left_8.toImageMsg());

	// publish right greyscale image
        cv_bridge::CvImage right_8;
        g_greyscale_image_left.copyTo(right_8.image);
        right_8.header.frame_id  = "guidance";
        right_8.header.stamp	 = ros::Time::now();
        right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;
        right_image_pub.publish(right_8.toImageMsg());
    }*/

    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
       /* printf( "frame index: %d, stamp: %d\n", imu_data->frame_index, imu_data->time_stamp );
        printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
 	
    	// publish imu data
	geometry_msgs::TransformStamped g_imu;
	g_imu.header.frame_id = "guidance";
	g_imu.header.stamp    = ros::Time::now();
	g_imu.transform.translation.x = imu_data->acc_x;
	g_imu.transform.translation.y = imu_data->acc_y;
	g_imu.transform.translation.z = imu_data->acc_z;
	g_imu.transform.rotation.x = imu_data->q[0];
	g_imu.transform.rotation.y = imu_data->q[1];
	g_imu.transform.rotation.z = imu_data->q[2];
	g_imu.transform.rotation.w = imu_data->q[3];
	imu_pub.publish(g_imu);
    }

    */




    imu_p.header.stamp = ros::Time::now();
    imu_p.header.frame_id = "imu";
    imu_p.orientation.x = imu_data->q[0];
    imu_p.orientation.y = imu_data->q[1];
    imu_p.orientation.z = imu_data->q[2];
    imu_p.orientation.w = imu_data->q[3];
imu_p.linear_acceleration.x = imu_data->acc_x;
imu_p.linear_acceleration.y = imu_data->acc_y;
imu_p.linear_acceleration.z = imu_data->acc_z;
  double roll, pitch, yaw;

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imu_p.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  //printf( "roll:%f pitch:%f yaw:%f\n", roll, pitch, yaw );
 //printf( "imu_data->acc_x:%f imu_data->acc_y:%f imu_data->acc_z:%f\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z);
/*
if (se=="abc") orientation.setRPY(roll, pitch, yaw);
else 
{


if(se=="acb") orientation.setRPY(roll, yaw, pitch);
 else if(se=="bac") orientation.setRPY(pitch, roll, yaw);
  else if(se=="bca") orientation.setRPY(pitch, yaw, roll);
   else if(se=="cab") orientation.setRPY(yaw, roll, pitch);
    else if(se=="cba") orientation.setRPY(yaw, pitch, roll);

/*
yaw=0;

if(se=="acb") orientation.setRPY(roll, -pitch, yaw);
 else if(se=="bac") orientation.setRPY(roll, pitch, -yaw);
  else if(se=="bca") orientation.setRPY(roll, -pitch, -yaw);
   else if(se=="cab") orientation.setRPY(-roll, -pitch, yaw);
    else if(se=="cba") orientation.setRPY(-roll, pitch, -yaw);
else if(se=="aaa") orientation.setRPY(-roll, -pitch, -yaw);
else if(se=="bbb") orientation.setRPY(-roll, pitch, yaw);

else if(se=="ccc") orientation.setRPY(roll, pitch, yaw);

* /
 tf::quaternionTFToMsg(orientation, imu_p.orientation);
}
/*
imu_p.linear_acceleration.x = imu_data->acc_x;
imu_p.linear_acceleration.y = imu_data->acc_y;
imu_p.linear_acceleration.z = imu_data->acc_z;

imu_p.orientation_covariance[0]=0.0012250000000000002;
imu_p.orientation_covariance[4]=0.0012250000000000002;
imu_p.orientation_covariance[8]=0.0012250000000000002;
imu_p.linear_acceleration_covariance[0]=0.00031329000000000003;
imu_p.linear_acceleration_covariance[4]=0.00031329000000000003;
imu_p.linear_acceleration_covariance[8]=0.00031329000000000003;
imu_p.angular_velocity_covariance[0]=6.25e-06;
imu_p.angular_velocity_covariance[4]=6.25e-06;
imu_p.angular_velocity_covariance[8]=6.25e-06;


  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    
*/

imu_m.publish(imu_p);
    }










    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
if(flag)
{
printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
       printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
}
       // printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
        //printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
	
	// publish velocity
	geometry_msgs::Vector3Stamped g_vo;
	g_vo.header.frame_id = "guidance";
	g_vo.header.stamp    = ros::Time::now();
	g_vo.vector.x = 0.001f * vo->vx;
	g_vo.vector.y = 0.001f * vo->vy;
	g_vo.vector.z = 0.001f * vo->vz;
	velocity_pub.publish(g_vo);
if(abs(vo->vx) >  0) flag=false;
    }

    /* obstacle distance
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
        printf( "frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp );
        printf( "obstacle distance:" );
        for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
        {
            printf( " %f ", 0.01f * oa->distance[i] );
        }
        printf( "\n" );

	// publish obstacle distance
	sensor_msgs::LaserScan g_oa;
        g_oa.ranges.resize(5);
	g_oa.header.frame_id = "guidance";
	g_oa.header.stamp    = ros::Time::now();
	g_oa.ranges[0] = 0.01f * oa->distance[0];
	g_oa.ranges[1] = 0.01f * oa->distance[1];
	g_oa.ranges[2] = 0.01f * oa->distance[2];
	g_oa.ranges[3] = 0.01f * oa->distance[3];
	g_oa.ranges[4] = 0.01f * oa->distance[4];
	obstacle_distance_pub.publish(g_oa);
    }*/

    /* ultrasonic
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
        printf( "frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
        for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
        {
            printf( "ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
        }
	
	// publish ultrasonic data
	sensor_msgs::LaserScan g_ul;
        g_ul.ranges.resize(5);
        g_ul.intensities.resize(5);
	g_ul.header.frame_id = "guidance";
	g_ul.header.stamp    = ros::Time::now();
	g_ul.ranges[0] = 0.001f * ultrasonic->ultrasonic[0];
	g_ul.ranges[1] = 0.001f * ultrasonic->ultrasonic[1];
	g_ul.ranges[2] = 0.001f * ultrasonic->ultrasonic[2];
	g_ul.ranges[3] = 0.001f * ultrasonic->ultrasonic[3];
	g_ul.ranges[4] = 0.001f * ultrasonic->ultrasonic[4];
	g_ul.intensities[0] = 1.0 * ultrasonic->reliability[0];
	g_ul.intensities[1] = 1.0 * ultrasonic->reliability[1];
	g_ul.intensities[2] = 1.0 * ultrasonic->reliability[2];
	g_ul.intensities[3] = 1.0 * ultrasonic->reliability[3];
	g_ul.intensities[4] = 1.0 * ultrasonic->reliability[4];
	ultrasonic_pub.publish(g_ul);
    }*/

    g_lock.leave();
    g_event.set_event();

    return 0;
}

void imu_callback(const sensor_msgs::Imu& g_imu)
{
    //imu_p.angular_velocity.x = g_imu.angular_velocity.x;
   // imu_p.angular_velocity.y = g_imu.angular_velocity.y;
    //imu_p.angular_velocity.z = g_imu.angular_velocity.z;
}



#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); printf( "error code:%d,%s %d\n", err_code, __FILE__, __LINE__ ); return -1;}}

int main(int argc, char* argv[1])
{
    /* initialize ros */
//se= argv[1];
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    //depth_image_pub        = my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
    //left_image_pub         = my_node.advertise<sensor_msgs::Image>("/guidance/left_image",1);
    //right_image_pub        = my_node.advertise<sensor_msgs::Image>("/guidance/right_image",1);
    imu_pub  		   = my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub  	   = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    imu_sub               = my_node.subscribe("/m100/imu", 1, imu_callback);//订阅m100IMU数据
    imu_m		= my_node.advertise<sensor_msgs::Imu>("/microstrain/imu", 5);//发布IMU数据

    //obstacle_distance_pub  = my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
    //ultrasonic_pub  	   = my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);
    /* select data */
   // select_greyscale_image(CAMERA_ID, true);
    //select_greyscale_image(CAMERA_ID, false);
    //select_depth_image(CAMERA_ID);
    select_imu();
    //select_ultrasonic();
   // select_obstacle_distance();
    select_velocity();
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);
    //std::cout << "start_transfer" << std::endl;

    while (ros::ok())
    {
  	if (key != 0)
	{
 	    err_code = stop_transfer();
    	    RETURN_IF_ERR(err_code);
	    reset_config();

       // if (key == 'q') break;
       // if (key == 'w') CAMERA_ID = e_vbus1;
       // if (key == 'd') CAMERA_ID = e_vbus2;
       // if (key == 'x') CAMERA_ID = e_vbus3;
       // if (key == 'a') CAMERA_ID = e_vbus4;
         //   if (key == 's') CAMERA_ID = e_vbus5;

      //  select_greyscale_image(CAMERA_ID, true);
          //  select_greyscale_image(CAMERA_ID, false);
           // select_depth_image(CAMERA_ID);
	    
	    err_code = start_transfer();
            RETURN_IF_ERR(err_code);
	}
        ros::spinOnce();
    }

    /* release data transfer */
    err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
   // std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);

    return 0;
}
