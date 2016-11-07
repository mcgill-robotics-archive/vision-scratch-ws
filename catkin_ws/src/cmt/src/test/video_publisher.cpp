
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <iostream>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

using cv::VideoCapture;

using namespace std;

int main(int argc, char **argv) {


	const int ros_topic_cmd = 1000;

	struct option longopts[] = {
		{"ros-topic", required_argument, 0, ros_topic_cmd}
	};

	string ros_topic = "";
	int index=0, c;
	while ((c  = getopt_long(argc, argv, "v", longopts, &index)) != -1) {
 		switch(c) {
 			case ros_topic_cmd:
 				ros_topic = string(optarg);
 				break;
 		}
	}

	if (ros_topic.length() == 0) {
		cerr << "Missing argument --ros-topic" << endl;
		return 0;
	}

	ros::init(argc, argv, "video_publisher");

	ros::NodeHandle handle;
	image_transport::ImageTransport transport(handle);
	image_transport::Publisher publisher = transport.advertise(ros_topic, 1);

	VideoCapture cap;
	cap.open(0);

	if (!cap.isOpened()) {
		cerr << "Unable to open Video Capture!" << endl;
		return 0;
	}

	ros::start();

	while (ros::ok()) {

		cv::Mat image;
		cap >> image;

		cv_bridge::CvImage out_msg;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = image;

 		publisher.publish(out_msg.toImageMsg());
	
	}

	ros::shutdown();
	return 0;

}