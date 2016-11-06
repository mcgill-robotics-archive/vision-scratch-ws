
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

class ImageStream {

	string ros_topic;

	unique_ptr<VideoCapture> video;
	unique_ptr<RosCapture> ros_cap;

	cv::Mat prev_frame;

	StreamSource source;

public:

	// Defaults to using an OpenCV image capture in the case that no topic is provided
	ImageStream(string ros_topic = "");

	// Default argument for VideoCapture open function
	void open(int arg = 0);
	void open(string video_path);
	bool isOpened();

	double get(int arg);
	void set(int arg, double val);

	const cv::Mat &getImage();

	ImageStream &operator >>(cv::Mat &mat);

};