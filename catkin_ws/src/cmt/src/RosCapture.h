
#include "cv_bridge/cv_bridge.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <string>
#include <list>
#include <mutex>

using std::string;
using std::list;
using std::mutex;

class RosCapture {

	ros::NodeHandle handle;
	image_transport::ImageTransport transport;
	image_transport::Subscriber image_sub;

	list<cv::Mat> image_queue;

	size_t queue_size;
	bool opened;

	mutex mu;

	void imageCallback(const sensor_msgs::ImageConstPtr &msg);

	void enqueueImage(const cv::Mat &image);

public:

	RosCapture(size_t queue_size = 3);

	void open(string ros_topic);
	bool isOpened() const;

	cv::Mat dequeueImage();

};