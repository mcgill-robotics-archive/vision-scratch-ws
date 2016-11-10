

#include "RosCapture.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <memory>
#include <string>

using cv::VideoCapture;
using std::unique_ptr;

enum StreamSource {
	Video,
	Topic
};


/*
 *	Abstracts away the capturing of images from either an OpenCV image capture or a rostopic streaming from a live run or rosbag.
 *
 *	example:
 *	
 *		ImageStream stream("/vision/webcam");
 *
 *		cv::Mat image;
 *		stream >> image;
 *	
 */

class ImageStream {

	string ros_topic;

	// Pointers to an OpenCV VideoCapture or RosCapture (reads from a topic).
	unique_ptr<VideoCapture> video;
	unique_ptr<RosCapture> ros_cap;

	// Stores the previous frame.
	cv::Mat prev_frame;

	// Stores which source the images come from (video / topic).
	StreamSource source;

public:

	// Defaults to using an OpenCV image capture in the case that no topic is provided
	ImageStream(string ros_topic = "");

	// Opens the image source.
	void open(int arg = 0);
	void open(string video_path);

	bool isOpened();

	// Wraps around VideoCapture.get() and .set() used by the OpenCV api.
	// These two methods serve no purpose in the case of using a ros topic as input.
	double get(int arg);
	void set(int arg, double val);

	// gets an Image from the associated stream (video or ros topic).
	const cv::Mat &getImage();

	// Input operator overload. 
	ImageStream &operator >>(cv::Mat &mat);

};