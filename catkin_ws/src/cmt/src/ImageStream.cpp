
#include "ImageStream.h"

ImageStream::ImageStream(std::string ros_topic) {
	
	// Set source enumeration to the appropriate source depending on whether a ros_topic is provided.
	source = ros_topic.length() > 0 ? Topic : Video;

}


double ImageStream::get(int arg) {
	return video.get(arg);
}
void ImageStream::set(int arg, double val) {
	video.set(arg, val);
}

void ImageStream::open(int arg) {

	// arg defaults to 0 (selects the default image capture device on the machine).
	if (source == Video) {
		video.open(arg);
	}

}

void ImageStream::open(std::string video_path) {

	if (source == Video) {
		video.open(video_path);
	}
	
}

bool ImageStream::isOpened() {

	if (source == Video) {
		return video.isOpened();
	}

	return false;
}

cv::Mat ImageStream::getImage() {

	cv::Mat image;
	if (source == Video) {

		video >> image;

	}

	return image;

}

ImageStream &ImageStream::operator >>(cv::Mat &mat) {
	mat = this->getImage();
	return *this;
}
