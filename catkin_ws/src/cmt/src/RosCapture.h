
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


/*
 *	Handles subscribing to a ros topic and converting ROS Image messages to cv::Mat format (BGR8).
 */

class RosCapture {

	// ROS Image Transport definitions.
	ros::NodeHandle handle;
	image_transport::ImageTransport transport;
	image_transport::Subscriber image_sub;

	// A queue of images which the subscriber has received.
	list<cv::Mat> image_queue;

	// The maximum size of the queue.
	size_t queue_size;

	bool opened;

    // A std::mutex to synchronize access to the queue (avoids race conditions between main thread and subscriber thread).
	mutex mu;

	// Callback for the subscriber to use.
	void imageCallback(const sensor_msgs::ImageConstPtr &msg);

	// Enqueues an image to the image queue.
	void enqueueImage(const cv::Mat &image);

public:

	RosCapture(size_t queue_size = 3);

	// Opens the subscriber with a given topic.
	void open(string ros_topic);
	bool isOpened() const;

	// Gets an image from the queue.
	cv::Mat dequeueImage();

};