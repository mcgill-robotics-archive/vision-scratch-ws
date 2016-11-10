
#include "ImageStream.h"

using std::string;


// Constructor
ImageStream::ImageStream(string ros_topic)
	: ros_topic(ros_topic)
{
	
	// Set source enumeration to the appropriate source depending on whether a ros_topic is provided or not.
	source = ros_topic.length() > 0 ? Topic : Video;

	if (source == Topic) {

		// Initialize the Ros Capturer
		ros_cap = unique_ptr<RosCapture>(new RosCapture());
	} else {

		// Initialize the Video Capturer
		video = unique_ptr<VideoCapture>(new VideoCapture());
	}

}


// Wrapper around the cv::VideoCapture method of the same name
double ImageStream::get(int arg) {
	if (source != Video)
		return 0.0;
	return video->get(arg);
}

// Wrapper around the cv::VideoCapture method of the same name
void ImageStream::set(int arg, double val) {
	if (source == Video)
		video->set(arg, val);
}


// Opens the video source.
void ImageStream::open(int arg) {

	if (source == Video) {
		// arg defaults to 0 (selects the default image capture device on the machine).
		video->open(arg);
	} else {
		ros_cap->open(ros_topic);
	}

}

// Opens the video source from a video file supported by OpenCV VideoCapture.
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

// Gets an image from the respective image source, also storing it into prev_frame.
const cv::Mat &ImageStream::getImage() {

	if (source == Video) {
		// Store the video into the previous frame.
		*video >> prev_frame;
	} else {

		// Dequeue an image from the RosCapture's image queue.
		cv::Mat img = ros_cap->dequeueImage();
		if (!img.empty())
			// If it isn't empty (will be empty if no images are in the queue) then save it into prev_frame.
			prev_frame = img;
	}

	// Return the image.
	return prev_frame;

}

// Input operator >>
ImageStream &ImageStream::operator >>(cv::Mat &mat) {
	mat = this->getImage();
	return *this;
}
