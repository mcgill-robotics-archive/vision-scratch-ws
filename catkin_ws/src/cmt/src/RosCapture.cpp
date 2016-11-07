
#include "RosCapture.h"

#include <thread>
#include <utility>

using std::string;
using std::lock_guard;
using std::mutex;
using std::size_t;

RosCapture::RosCapture(size_t queue_size)
	: queue_size(queue_size),
	  opened(false),
	  transport(handle)
{
	image_queue.resize(queue_size);
}

void RosCapture::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

	cv_bridge::CvImagePtr cv_ptr;
	try 
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} 
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	enqueueImage(cv_ptr->image);

}

void RosCapture::open(string ros_topic) {
	image_sub = transport.subscribe(ros_topic, 1, &RosCapture::imageCallback, this);
	opened = true;
}

bool RosCapture::isOpened() const {
	return opened;
}

void RosCapture::enqueueImage(const cv::Mat &image) {
	
	// Synchronizes
	lock_guard<mutex> lockg(mu);

	if (image_queue.size() < queue_size) {
		image_queue.emplace_front(image);
	}
	else {
		image_queue.back() = image;
	}

}

cv::Mat RosCapture::dequeueImage() {

	lock_guard<mutex> lockg(mu);

	if (image_queue.size() > 0) {
		cv::Mat image = image_queue.back();
		image_queue.pop_back();
		return image;
	} else {
		// Default
		return cv::Mat();
	}

}
