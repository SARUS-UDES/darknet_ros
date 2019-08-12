

#pragma once

// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// Configurable parameters
#include <darknet_ros/DarknetRosParamConfig.h>
#include <dynamic_reconfigure/server.h>

// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

// Import libraries for message syncronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Darknet.
#ifdef GPU
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#endif

extern "C" {
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "darknet_ros/image_interface.h"
#include <sys/time.h>
}

extern "C" image mat_to_image(cv::Mat m);
extern "C" cv::Mat image_to_mat(image im);
// extern "C" void make_window(char *name, int w, int h, int fullscreen);
// extern "C" int show_image(image p, const char *name, int ms);

namespace darknet_ros {

//! Bounding box of the detected object.
typedef struct
{
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

class YoloObjectDetector
{
 public:
  /*!
   * Constructor.
   */
  explicit YoloObjectDetector(ros::NodeHandle nh);

  /*!
   * Destructor.
   */
  ~YoloObjectDetector();

  // Initializing a dynamic parameter server 
  dynamic_reconfigure::Server<darknet_ros::DarknetRosParamConfig> cfg_server;
  dynamic_reconfigure::Server<darknet_ros::DarknetRosParamConfig>::CallbackType cfg_f;
  void cfg_callback(darknet_ros::DarknetRosParamConfig& config, uint32_t level);

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Callback which creates a copy of the camera image recieved
   * @param[in] camera msg image pointer
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& camMsg);

  /*!
   * Check for objects action goal callback.
   */
  void checkForObjectsActionGoalCB();

  /*!
   * Check for objects action preempt callback.
   */
  void checkForObjectsActionPreemptCB();

  /*!
   * Check if a preempt for the check for objects action has been requested.
   * @return false if preempt has been requested or inactive.
   */
  bool isCheckingForObjects() const;

  /*!
   * Publishes the detection image.
   * @return true if successful.
   */
  bool publishDetectionImage();

  //! Typedefs.
  typedef actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction> 
                                                  CheckForObjectsActionServer;
  typedef std::shared_ptr<CheckForObjectsActionServer> CheckForObjectsActionServerPtr;

  // Typedefs to create approximate time policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                          sensor_msgs::Image> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! Class labels.
  int numClasses_;
  std::vector<std::string> classLabels_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber camSubscriber_;

  //! Check for objects action server.
  CheckForObjectsActionServerPtr checkForObjectsActionServer_;

  //! ROS publishers.
  ros::Publisher boundingBoxesPublisher_;
  ros::Publisher detectionImagePublisher_; 
  ros::Publisher objectPublisher_;                     

  //! Detected objects.
  std::vector<std::vector<RosBox_> > rosBoxes_;
  std::vector<int> rosBoxCounter_;
  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;
  
  // Parameters related to camera image
  int frameWidth_;
  int frameHeight_;

  // frame for all publishers to publish in
  std::string frameToPublishIn_;

  // Frames per second that the node is running at
  float fps_ = 0;

  // Threshold before which yolo doesn't include a bounding box
  double YOLO_THRESH;

  // Whether to use the image header for the header of the result bounding box message  
  bool USE_IMAGE_HEADER_TIMESTAMP;

  // To store bounding boxes which are then added to the result buffer
  darknet_ros_msgs::BoundingBox boundingBox_;

  // Yolo running on thread.
  std::thread yoloThread_;

  // Variables to store copy of messages from the callback
  std_msgs::Header imageHeader_;
  cv::Mat camImageCopy_;

  // Buffer to store camera images
  image buff_[3];
  
  // Buffer to store headers of messages recieved
  std_msgs::Header headerBuff_[3];

  // Mutexes
  boost::shared_mutex mutexImageCallback_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;

  bool isNodeRunning_ = true;
  boost::shared_mutex mutexNodeStatus_;

  int actionId_;
  boost::shared_mutex mutexActionStatus_;

  // Darknet variables
  char **demoNames_;
  image **demoAlphabet_;
  int demoClasses_;
  network *net_;

  // Below variables are related to darknet inference. For a better understanding
  // of how inference works, have a look at darknet/src/demo.c
  image buffLetter_[3];
  int buffId_[3];
  int buffIndex_ = 0;
  float demoHier_ = .5;

  int demoFrame_ = 3;
  float **predictions_;
  int demoIndex_ = 0;
  int demoDone_ = 0;
  float *lastAvg2_;
  float *lastAvg_;
  float *avg_;
  int demoTotal_ = 0;
  double demoTime_;
  RosBox_ *roiBoxes_;
  
  // The below functions are related to darknet inference
  int sizeNetwork(network *net);

  void rememberNetwork(network *net);
  
  detection *avgPredictions(network *net, int *nboxes);

  /*!
   * Function which contains the detect, fetch, and publish threads
   */
  void yolo();

  /*
   * Used at the beginning to check whether an image has been recieved. This is used to 
   * start the detect, fetch, and publish threads
   * @return whether an image has been recieved in the callback
   */
  bool getImageStatus(void);

  /*
   * Returns the boolean isNodeRunning_, which is set to False when the node is killed
   * @return the boolean isNodeRunning_ 
   */
  bool isNodeRunning(void);
  
  /*
   * All results from the detection thread are put into the right message format and 
   * published in this function
   */  
  void *publishInThread();

  /*
   * Inference of bounding boxes happens here
   */
  void *detectInThread();

  /*
   * Fetches messages from the callback and adds them to the buffers
   */
  void *fetchInThread();

  // Sets up the network for inference
  void setupNetwork(char *cfgfile, char *weightfile, char *datafile,
                    char **names, int classes, int avg_frames, float hier);
  
};

} /* namespace darknet_ros*/
