# CMT Package

This catkin package contains the CMT vision algorithm which comes from
https://github.com/gnebehay/CppMT. 

# packaged executables/nodes
### cmt_node
A slightly modified version of the original implementation. However it can accept input from either a webcam or a ros topic that streams image (provided the format is compatible with OpenCV BGR8 encoding). For now this node only outputs to webcam and receives bounding box input from the user using a simple GUI.
##### Additional Arguments:
ros-topic: Not required. If provided cmt will get its input from the respective ros topic.
##### Example Usage:
`rosrun cmt cmt_node --ros-topic /vision/webcam`

Kill: `rosnode kill /cmt_node`
### cmt_webcam
Perfect for testing, this node simply publishes a webcam feed to a provided ros topic.
##### Arguments:
ros-topic: Required. The topic to use when publishing webcam frames.

##### Example Usage:
`rosrun cmt cmt_webcam --ros-topic /vision/webcam`

Kill: `rosnode kill /cmt_webcam`