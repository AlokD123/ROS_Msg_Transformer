#include <sensor_msgs/NavSatFix.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64.h>
#include <string>

//#define DEBUG_

class MsgTransformer {
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    std::string subTopic_, pubTopic_; // TO DO: update to arbitary message type
                                      // and arbitrary fields for alteration
    uint32_t pubQueueSize_, subQueueSize_;
    std::vector<double> positionCovariance_; // TO DO: update to arbitrary fields

    // One possible callback for the subscriber in the class. Publishes to
    // ALTERNATE topic with transformed msg
    void subscribeAndPublish(const sensor_msgs::NavSatFix &gps_msg);

    // Another possible callback for the subscriber in the class. Just changes msg
    // by reference void subscribe(sensor_msgs::NavSatFix&); //TO DO: update to
    // arbitary message type

    // Transform the msg, return a copy with updated fields
    sensor_msgs::NavSatFix alterMsg(const sensor_msgs::NavSatFix &);

  public:
    MsgTransformer(ros::NodeHandle nh, std::string subTopic,
                  uint32_t pubQueueSize, uint32_t subQueueSize,
                  std::vector<double> positionCovariance);

    // Change publish options
    void setPublishOptions(std::string topic); // TO DO: update to arbitrary
                                              // message type (another argument)
    // Change subscribe options
    void
    setSubscribeOptions(std::string topic); // TO DO: update to arbitrary message
                                            // type (another argument)
};

MsgTransformer::MsgTransformer(ros::NodeHandle nh, std::string subTopic,
                               uint32_t pubQueueSize, uint32_t subQueueSize,
                               std::vector<double> positionCovariance)
    : nh_(nh), subTopic_(subTopic), pubQueueSize_(pubQueueSize),
      subQueueSize_(subQueueSize), positionCovariance_(positionCovariance) {
  pub_ = nh.advertise<sensor_msgs::NavSatFix>((subTopic + "_alt").c_str(), //Publish to another topic with suffix "_alt"
                                          pubQueueSize);
  sub_ = nh.subscribe(subTopic.c_str(), subQueueSize,
                      &MsgTransformer::subscribeAndPublish, this);

  #ifdef DEBUG_
    ROS_INFO("Constructed");
  #endif
};

void MsgTransformer::setPublishOptions(std::string pubTopic) {
  // MsgTransformer::pub_.topic_ = pubTopic.c_str(); // TO DO: not currently
  // present
}
void MsgTransformer::setSubscribeOptions(std::string subTopic) {
  // MsgTransformer::sub_.topic_ = subTopic.c_str(); // TO DO: not currently
  // present
}

sensor_msgs::NavSatFix
MsgTransformer::alterMsg(const sensor_msgs::NavSatFix &gps_msg) {
  // TO DO: generalize alterations to message
  // Currently, copy to new msg and overwrite covariance fields
  sensor_msgs::NavSatFix altOdomMsg =
      gps_msg; // Note: for pointer* instead of reference&, would need to
                // explicitly dereference

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

    memcpy(&(altOdomMsg.position_covariance), position_cov,
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

    pub_.publish(newMsg);
}

int main(int argc, char **argv) {
  // Initiate node and set handle
  ros::init(argc, argv, "msg_transformer_gps");
  ros::NodeHandle nh;

  ROS_INFO("Initiated msg_transformer node");

  // Create MsgTransformer object
  std::string subTopic;
  int pubQueueSize, subQueueSize;
  std::vector<double> positionCovariance;
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

  MsgTransformer mt = MsgTransformer(nh, subTopic, pubQueueSize, subQueueSize,
                                     positionCovariance);
  ROS_INFO("Created msg_transformer object");

  // Start subscribing and publishing to alternate
  while (ros::ok()) { // Alternate: ros::Rate(_), rate.sleep();
    ros::spinOnce();
  } // The above is equivalent to ros::spin();
}