// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh),
      imageTransport_(nodeHandle_),
      numClasses_(0),
      classLabels_(0),
      rosBoxes_(0),
      rosBoxCounter_(0)
{
  ROS_DEBUG("[darknet_ros] Node started.\n");

  // Read parameters from config file.
  if (!readParameters()) 
  {
    ros::requestShutdown();
  }
  init();
}

YoloObjectDetector::~YoloObjectDetector()
{
  ROS_DEBUG("[darknet_ros] Killing node");
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }

  // Complete all running processes
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters()
{
  // Set vector sizes
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init()
{
  ROS_DEBUG("[darknet_ros] init().\n");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  nodeHandle_.param("yolo_model/threshold/value", YOLO_THRESH, 0.5);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                      std::string("yolov2-tiny.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, 
                      std::string("yolov2-tiny.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * 
                    sizeof(char*));

  for (int i = 0; i < numClasses_; i++) 
  {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, detectionNames, numClasses_, 1, 0.5);
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

  // Sleep here to ensure network is properly set up
  sleep(2);

  // Variables to store parameters
  std::string cameraTopicName;             // Topic which camera images are published to
  int cameraQueueSize;                 // Queue size of camera topic subscribed to

  std::string objectDetectorTopicName; 
  int objectDetectorQueueSize;
  bool objectDetectorLatch;

  std::string boundingBoxesTopicName;      // Topic to publish boudning boxes to
  int boundingBoxesQueueSize;          // Queue size of publishing to boundingBoxesTopicName
  bool boundingBoxesLatch;             // Enable latching of boundingBoxesTopic

  std::string detectionImageTopicName;     // Topic to publish detection images to
  int detectionImageQueueSize;         // Queue size of publishing to detectionImageTopic
  bool detectionImageLatch;            // Enable latching of detectionImage topic


  // Assigning parameters from the darknet_ros.yaml file in perception_meta/cfg
  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);

  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName,
                    std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);

  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                    std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);

  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

  // Initialize publishers
  objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName, 
                        objectDetectorQueueSize, objectDetectorLatch);
  boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
                        boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(
         detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

  // Initialize subscriber to camera
  camSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize,
                                               &YoloObjectDetector::cameraCallback, this);
  
  // Configure the configuration callback
  cfg_f = boost::bind(&YoloObjectDetector::cfg_callback, this, _1, _2);
  cfg_server.setCallback(cfg_f);

  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName,
                    std::string("check_for_objects"));
  checkForObjectsActionServer_.reset(
      new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(
      boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();

  ROS_DEBUG("[darknet_ros] finished initalizing publishers and subscribers.\n");
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& camMsg)
{
  ROS_DEBUG("[cameraCallback] Camera image received.\n");

  cv_bridge::CvImagePtr camImage;

  try 
  {
    camImage = cv_bridge::toCvCopy(camMsg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("[darknet_ros] Unable to copy camera image in darknet_ros.\
                  cv exception: %s", e.what());
    return;
  }

  /** Create copies of images and information from the messages that we will need later
  Variable camImageCopy_ is accessed by other parts of the code. Using mutexs ensures that 
  no other part of our node is accessing these variables at the same time we're assigning 
  them. This ensure no segmentation faults occur.
  */
  if (camImage) 
  {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = camMsg->header;
      camImageCopy_ = camImage->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }

    frameWidth_ = camMsg->width;
    frameHeight_ = camMsg->height;
    frameToPublishIn_ = (*camMsg).header.frame_id;
  }
}

void YoloObjectDetector::checkForObjectsActionGoalCB()
{
  ROS_DEBUG("[checkForObjectsActionGoalCB] Start check for objects action.\n");

  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr =
      checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr camImage;

  try 
  {
    camImage = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } 
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("[darknet_ros] Unable to copy image in checkForObjectsActionGoalCB().\
                 cv_bridge exception: %s", e.what());
    return;
  }

  if (camImage) 
  {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = camImage->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_);
      actionId_ = imageActionPtr->id;
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = camImage->image.size().width;
    frameHeight_ = camImage->image.size().height;
  }
}

void YoloObjectDetector::checkForObjectsActionPreemptCB()
{
  ROS_DEBUG("[checkForObjectsActionPreemptCB] Preempt check for objects action.\n");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const
{
  return (ros::ok() && checkForObjectsActionServer_->isActive()
      && !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::publishDetectionImage()
{
  if (detectionImagePublisher_.getNumSubscribers() < 1) 
  {
    return false;
  }

  cv_bridge::CvImage cvImage;
  cvImage.image = image_to_mat(buff_[(buffIndex_ + 1)%3]);
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = frameToPublishIn_;
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  return true;
}

int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;

  for(i = 0; i < net->n; ++i)
  {
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
    {
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  int i;
  int count = 0;

  for(i = 0; i < net->n; ++i)
  {
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
    {
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, 
                                            sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);

  for(j = 0; j < demoFrame_; ++j)
  {
    axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  }

  for(i = 0; i < net->n; ++i)
  {
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
    {
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }

  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, YOLO_THRESH, demoHier_, 0, 1, nboxes);
  return dets;
}

void *YoloObjectDetector::detectInThread()
{
  ROS_DEBUG("[detectInThread] detecting bounding boxes\n");
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float *X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float *prediction = network_predict(net_, X);

  rememberNetwork(net_);
  detection *dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  ROS_DEBUG("[detectInThread] finish detecting bounding boxes\n");
  
  image display = buff_[(buffIndex_+2) % 3];
  draw_detections(display, dets, nboxes, YOLO_THRESH, demoNames_, demoAlphabet_, demoClasses_);

  // extract the bounding boxes and send them to ROS
  int i, j;
  int count = 0;

  for (i = 0; i < nboxes; ++i) 
  {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0)
      xmin = 0;
    if (ymin < 0)
      ymin = 0;
    if (xmax > 1)
      xmax = 1;
    if (ymax > 1)
      ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) 
    {
      if (dets[i].prob[j]) 
      {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) 
        {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = dets[i].prob[j];
          
          count++;
        }
      }
    }
  }

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) 
  {
    roiBoxes_[0].num = 0;
  } 
  else 
  {
    roiBoxes_[0].num = count;
  }

  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  return 0;
}

void *YoloObjectDetector::fetchInThread()
{
  // Free image memory
  free_image(buff_[buffIndex_]);

  ROS_DEBUG("[fetchInThread] fetching camera and depth image and adding to buffer.\n");

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    buff_[buffIndex_] = mat_to_image(camImageCopy_);
    buffId_[buffIndex_] = actionId_;
    headerBuff_[buffIndex_] = imageHeader_;
  }

  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
  
  return 0;
}


void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, 
                      char **names, int classes, int avg_frames, 
                      float hier)
{
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoHier_ = hier;

  ROS_DEBUG("Setting up YOLO network\n");

  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo()
{
  const auto waitDuration = std::chrono::milliseconds((long)2000);

  while (!getImageStatus()) 
  {
    ROS_WARN("[darknet_ros] Waiting for image.\n");

    if (!isNodeRunning()) 
    {
      return;
    }

    std::this_thread::sleep_for(waitDuration);
  }

  std::thread detectThread;
  std::thread fetchThread;

  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float **) calloc(demoFrame_, sizeof(float*));

  for (i = 0; i < demoFrame_; ++i)
  {
    predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
  }

  avg_ = (float *) calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  // Since on the first cycle of the while loop we don't have images for all the
  // index in the buffer, we initialize all of them to the first depth image 
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    buff_[0] = mat_to_image(camImageCopy_);
  }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);

  headerBuff_[0] = imageHeader_;
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];

  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);

  int count = 0;

  while (!demoDone_) 
  {
    ROS_DEBUG("\n[yolo] new cycle \n\n");
    buffIndex_ = (buffIndex_ + 1) % 3;

    publishInThread();

    fetchThread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detectThread = std::thread(&YoloObjectDetector::detectInThread, this);

    fetchThread.join();
    detectThread.join();
    ++count;

    if (!isNodeRunning()) 
    {
      demoDone_ = true;
    }
  }
}

bool YoloObjectDetector::getImageStatus(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::isNodeRunning(void)
{
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void *YoloObjectDetector::publishInThread()
{
  ROS_DEBUG("[publishInThread] started.\n");
  
  if (!publishDetectionImage()) 
  {
    ROS_DEBUG("[publishInThread] Detection image has not been broadcasted.\n");
  }

  // Publish bounding boxes and detection result.
  int num = roiBoxes_[0].num;
  if (num > 0 && num <= 100) 
  {
    for (int i = 0; i < num; i++) 
    {
      for (int j = 0; j < numClasses_; j++) 
      {
        if (roiBoxes_[i].Class == j) 
        {
          rosBoxes_[j].push_back(roiBoxes_[i]);
          rosBoxCounter_[j]++;
        }
      }
    }

    std_msgs::Int8 msg;
    msg.data = num;
    objectPublisher_.publish(msg);

    // Iterate over each class
    for (int i = 0; i < numClasses_; i++) 
    {
      // Check if class has a bounding box
      if (rosBoxCounter_[i] > 0) 
      {
        // Iterate over the bounding boxes in each class
        for (int j = 0; j < rosBoxCounter_[i]; j++) 
        {
          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;
          
          boundingBox_.Class = classLabels_[i];
          boundingBox_.probability = rosBoxes_[i][j].prob;
          boundingBox_.xmin = xmin;
          boundingBox_.ymin = ymin;
          boundingBox_.xmax = xmax;
          boundingBox_.ymax = ymax;
          boundingBoxesResults_.bounding_boxes.push_back(boundingBox_);
        }
      }
    }
    
    ROS_DEBUG("[publishInThread] publishing results.\n");

    if (USE_IMAGE_HEADER_TIMESTAMP)
    {
      boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];
    }
    else 
    {
      boundingBoxesResults_.header.stamp = ros::Time::now();
    }
    boundingBoxesResults_.header.frame_id = frameToPublishIn_;
    boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];

    // Only publish bounding boxes if there are subscribers
    if (boundingBoxesPublisher_.getNumSubscribers() != 0) boundingBoxesPublisher_.publish(boundingBoxesResults_);
  } 
  else 
  {
    std_msgs::Int8 msg;
    msg.data = 0;
    objectPublisher_.publish(msg);
  }

  if (isCheckingForObjects()) 
  {
    ROS_DEBUG("[YoloObjectDetector] check for objects in image.\n");
    darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
    objectsActionResult.id = buffId_[0];
    objectsActionResult.bounding_boxes = boundingBoxesResults_;
    checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.\n");
  }

  // Clearing any data that was added to the results messages
  boundingBoxesResults_.bounding_boxes.clear();

  for (int i = 0; i < numClasses_; i++) 
  {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  ROS_DEBUG("[publishInThread] finished.\n");

  return 0;
}

void YoloObjectDetector::cfg_callback(darknet_ros::DarknetRosParamConfig &config, uint32_t level) 
{
  YOLO_THRESH = config.YOLO_THRESH;
  USE_IMAGE_HEADER_TIMESTAMP = config.USE_IMAGE_HEADER_TIMESTAMP;
}

} /* namespace darknet_ros*/

