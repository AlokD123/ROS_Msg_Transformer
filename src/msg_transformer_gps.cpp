#include <sensor_msgs/NavSatFix.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64.h>
#include <string>

//#define DEBUG_

class MsgTransformer {
/**
 * @brief This class transforms the fields of messages in ROS "on-the-fly" (i.e. by overwritting with custom values and republishing).
 * @note For a brief explanation of ROS and distributed messaging, refer to: http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
 * @todo Generalize this class to be for all types of messages, not just GPS messages (this application)
*/

  private:
    ros::NodeHandle nh_; //Handle for node in application, with which ROS master communicates
    ros::Publisher pub_;  //Publisher of transformed messages
    ros::Subscriber sub_; //Subscriber to raw messages

    std::string subTopic_, pubTopic_;         // Topics for raw messages (subscibed) and transformed messages
    uint32_t pubQueueSize_, subQueueSize_;    // Queue sizes for the publish and subscribe processes
    std::vector<double> positionCovariance_;  // New value for field in GPS message ("uncertainty in position"), populated during construction

    /**
     * @brief One possible callback for the subscriber in the class. Publishes to a renamed ALTERNATE topic with the transformed msg
     * @param gps_msg: raw message to be read, and a transformed message returned as a copy (see alterMsg())
    */
    void subscribeAndPublish(const sensor_msgs::NavSatFix &gps_msg);

    /**
     * @brief Another possible callback for the subscriber in the class. Changes msg directly, by reference
     * @param gps_msg: raw message to be transformed "in-place"
    */
    void subscribe(sensor_msgs::NavSatFix& gps_msg);


    /**
     * @brief Transform the msg, return a copy with updated fields
     * @param msg: raw message to be transformed
     * @return the transformed GPS message
    */
    sensor_msgs::NavSatFix alterMsg(const sensor_msgs::NavSatFix & msg);

  public:
    /**
     * @brief Construct a message transformer. Most params described above, except...
     * @param positionCovariance: new value of position covariance to be set
     * @param new_topic_suffix: a suffix for the new topic name, to distinguish from old one
     * @return the transformed GPS message
    */
    MsgTransformer(ros::NodeHandle nh, std::string subTopic,
                  uint32_t pubQueueSize, uint32_t subQueueSize,
                  std::vector<double> positionCovariance,
                  std::string new_topic_suffix);

    /**
     * @brief expose publisher parameters to change. Currently limited to...
     * @param topic: name of transformed message topic
    */
    void setPublishOptions(std::string topic); 
    /**
     * @brief expose subscriber parameters to change. Currently limited to...
     * @param topic: name of raw message topic
    */
    void setSubscribeOptions(std::string topic); 
};

MsgTransformer::MsgTransformer(ros::NodeHandle nh, std::string subTopic,
                               uint32_t pubQueueSize, uint32_t subQueueSize,
                               std::vector<double> positionCovariance,
                               std::string new_topic_suffix="_alt")
    : nh_(nh), subTopic_(subTopic), pubQueueSize_(pubQueueSize),
      subQueueSize_(subQueueSize), positionCovariance_(positionCovariance) {
  pub_ = nh.advertise<sensor_msgs::NavSatFix>((subTopic + new_topic_suffix).c_str(), //Publish to another topic, renamed using additional suffix
                                              pubQueueSize);
  sub_ = nh.subscribe(subTopic.c_str(), subQueueSize,
                      &MsgTransformer::subscribeAndPublish, this);

  #ifdef DEBUG_
    ROS_INFO("Constructed MsgTransformer");
  #endif
};

sensor_msgs::NavSatFix
MsgTransformer::alterMsg(const sensor_msgs::NavSatFix &gps_msg) {
  // Alters message
  // Currently, copies to new msg and overwrites covariance field
  sensor_msgs::NavSatFix altOdomMsg = gps_msg;

  #ifdef DEBUG_
    int i = 0;
    ROS_INFO("Size of cov array = %lu",
            sizeof(altOdomMsg.position_covariance) / sizeof(double));
    ROS_INFO("Original msg position cov[%d]=%f", i + 1, altOdomMsg.position_covariance[i]);
    ROS_INFO("position_covariance_[%d]=%f", i + 1, positionCovariance_[i]);
  #endif

    double *position_cov = &positionCovariance_[0]; // Convert to array for memcpy

  #ifdef DEBUG_
    ROS_INFO("position_cov[%d]=%f", i + 1, *(position_cov + i));
    assert(sizeof(altOdomMsg.position_covariance) == 9 * sizeof(double));
  #endif

    memcpy(&(altOdomMsg.position_covariance), position_cov, //Perform overwriting
          sizeof(altOdomMsg.position_covariance));

  #ifdef DEBUG_
    ROS_INFO("Altered msg. new_msg position cov[%d]=%f", i + 1,
            altOdomMsg.position_covariance[i]);
  #endif

    return altOdomMsg;
  }


  void MsgTransformer::subscribeAndPublish(const sensor_msgs::NavSatFix &gps_msg) {

    sensor_msgs::NavSatFix newMsg = alterMsg(gps_msg); // Alter the message

  #ifdef DEBUG_
    int j = 0;
    ROS_INFO("Before pub position cov[%d]=%f", j + 1, newMsg.position_covariance[j]);
  #endif

    pub_.publish(newMsg); //Publish new message
}

/**
 * @brief example driver for the MessageTransformer class
 * @param topic: name of transformed message topic
*/

int main(int argc, char **argv) {
  // Initiate node and set handle
  ros::init(argc, argv, "msg_transformer_gps");
  ros::NodeHandle nh;

  ROS_INFO("Initiated msg_transformer node");

  // Create MsgTransformer object
  std::string subTopic;
  int pubQueueSize, subQueueSize;
  std::vector<double> positionCovariance;     //Currently set in launch file using ROS parameter server
  nh.getParam("subscriber_topic", subTopic);
  #ifdef DEBUG_
    ROS_INFO("Got subscribe topic %s", subTopic.c_str());
  #endif
    nh.getParam("pub_queue_size", pubQueueSize);
  #ifdef DEBUG_
    ROS_INFO("Got pubQueueSize %i", pubQueueSize);
  #endif
    nh.getParam("sub_queue_size", subQueueSize);
    nh.getParam("position_cov", positionCovariance);
  #ifdef DEBUG_
    ROS_INFO("Got positionCovariance[0]= %f", positionCovariance.front());
  #endif

  MsgTransformer mt = MsgTransformer(nh, subTopic, pubQueueSize, subQueueSize, positionCovariance);
  ROS_INFO("Created msg_transformer object");

  // Start subscribing and publishing to alternate
  while (ros::ok()) { 
    ros::spinOnce(); //Sleep process, and just reawaken for the callback events
  } 
}