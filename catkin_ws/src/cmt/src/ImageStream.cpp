
#include "ImageStream.h"

using std::string;

ImageStream::ImageStream(string ros_topic)
	: ros_topic(ros_topic)
{
	
	// Set source enumeration to the appropriate source depending on whether a ros_topic is provided.
	source = ros_topic.length() > 0 ? Topic : Video;

	if (source == Topic) {
		// Initialize the Ros Capturer
		ros_cap = unique_ptr<RosCapture>(new RosCapture());
	} else {
		video = unique_ptr<VideoCapture>(new VideoCapture());
	}

}


double ImageStream::get(int arg) {
	if (source != Video)
		return 0.0;
	return video->get(arg);
}
void ImageStream::set(int arg, double val) {
	if (source == Video)
		video->set(arg, val);
}

void ImageStream::open(int arg) {

	// arg defaults to 0 (selects the default image capture device on the machine).
	if (source == Video) {
		video->open(arg);
	} else {
		ros_cap->open(ros_topic);
	}

}

void ImageStream::open(string video_path) {

	if (source == Video) {
		video->open(video_path);
	}
	
}

bool ImageStream::isOpened() {

	if (source == Video) {
		return video->isOpened();
	} else {
		return ros_cap->isOpened();
	}

	return false;
}

const cv::Mat &ImageStream::getImage() {

	if (source == Video) {
		*video >> prev_frame;
	} else {
		cv::Mat img = ros_cap->dequeueImage();
		if (!img.empty())
			prev_frame = img;
	}

	return prev_frame;

}

ImageStream &ImageStream::operator >>(cv::Mat &mat) {
	mat = this->getImage();
	return *this;
}
