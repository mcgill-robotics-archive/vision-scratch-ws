
#include "RosCapture.h"

#include <thread>
#include <utility>

using std::string;
using std::lock_guard;
using std::mutex;
using std::size_t;

RosCapture::RosCapture(size_t queue_size) : 
	queue_size(queue_size),
	opened(false),
	transport(handle)
{
	// Use the std::list resize feature to fix it's size to queue_size.
	image_queue.resize(queue_size);
}

void RosCapture::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

	// Code to convert from a ROS Image message (with any encoding as long as it's compatible with BGR8) to an openCV image with BGR8 encoding.
	cv_bridge::CvImagePtr cv_ptr;
	try 
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} 
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Once it's converted, enqueue the cv::Mat to the image queue.
	enqueueImage(cv_ptr->image);

}

void RosCapture::open(string ros_topic) {

	// Fires up the subscriber.
	image_sub = transport.subscribe(ros_topic, 1, &RosCapture::imageCallback, this);
	opened = true;
}

bool RosCapture::isOpened() const {
	return opened;
}

void RosCapture::enqueueImage(const cv::Mat &image) {
	
	// Lock the mutex with a std::lock_guard.
	// Avoids any race conditions to access the queue's resources.
	lock_guard<mutex> lockg(mu);

	if (image_queue.size() < queue_size) {
		// If the image queue is smaller than the max then just push the image to the back.
		image_queue.push_back(image);
	}
	else {
		// Otherwise replace the image at the back.
		image_queue.back() = image;
	}

}

cv::Mat RosCapture::dequeueImage() {

	// Lock the mutex with a std::lock_guard.
	// Avoids any race conditions to access the queue's resources.	lock_guard<mutex> lockg(mu);
	lock_guard<mutex> lockg(mu);

	if (image_queue.size() > 0) {
		// Take an image from the front.
		cv::Mat image = image_queue.front();
		image_queue.pop_front();
		return image;
	} else {
		// Default to an empty cv::Mat.
		return cv::Mat();
	}

}
