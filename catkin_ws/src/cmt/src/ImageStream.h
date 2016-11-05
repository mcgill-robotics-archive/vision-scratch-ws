
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

using cv::VideoCapture;

enum StreamSource {
	Video,
	Topic
};

class ImageStream {

	VideoCapture video;
	StreamSource source;

public:

	// Defaults to using an OpenCV image capture in the case that no topic is provided
	ImageStream(std::string ros_topic = "");

	// Default argument for VideoCapture open function
	void open(int arg = 0);
	void open(std::string video_path);
	bool isOpened();

	double get(int arg);
	void set(int arg, double val);

	cv::Mat getImage();

	ImageStream &operator >>(cv::Mat &mat);

};