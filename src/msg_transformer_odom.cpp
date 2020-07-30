#include <nav_msgs/Odometry.h>
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
  std::vector<double> poseCovariance_,
      twistCovariance_; // TO DO: update to arbitrary fields

  // One possible callback for the subscriber in the class. Publishes to
  // ALTERNATE topic with transformed msg
  void subscribeAndPublish(const nav_msgs::Odometry &odom_msg);

  // Another possible callback for the subscriber in the class. Just changes msg
  // by reference void subscribe(nav_msgs::Odometry&); //TO DO: update to
  // arbitary message type

  // Transform the msg, return a copy with updated fields
  nav_msgs::Odometry alterMsg(const nav_msgs::Odometry &);

public:
  MsgTransformer(ros::NodeHandle nh, std::string subTopic,
                 uint32_t pubQueueSize, uint32_t subQueueSize,
                 std::vector<double> poseCovariance,
                 std::vector<double> twistCovariance);

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
                               std::vector<double> poseCovariance,
                               std::vector<double> twistCovariance)
    : nh_(nh), subTopic_(subTopic), pubQueueSize_(pubQueueSize),
      subQueueSize_(subQueueSize), poseCovariance_(poseCovariance),
      twistCovariance_(twistCovariance) {
  pub_ = nh.advertise<nav_msgs::Odometry>((subTopic + "_alt").c_str(), //Publish to another topic with suffix "_alt"
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

nav_msgs::Odometry
MsgTransformer::alterMsg(const nav_msgs::Odometry &odom_msg) {
  // TO DO: generalize alterations to message
  // Currently, copy to new msg and overwrite covariance fields
  nav_msgs::Odometry altOdomMsg =
      odom_msg; // Note: for pointer* instead of reference&, would need to
                // explicitly dereference

#ifdef DEBUG_
  int i = 18;
  ROS_INFO("Size of cov array = %lu",
           sizeof(altOdomMsg.pose.covariance) / sizeof(double));
  ROS_INFO("Original msg pos cov[%d]=%f", i + 1, altOdomMsg.pose.covariance[i]);
  ROS_INFO("pose_covariance_[%d]=%f", i + 1, poseCovariance_[i]);
#endif

  double *pose_cov = &poseCovariance_[0]; // Convert to array for memcpy
  double *twist_cov = &twistCovariance_[0];

#ifdef DEBUG_
  ROS_INFO("pose_cov[%d]=%f", i + 1, *(pose_cov + i));
  assert(sizeof(altOdomMsg.pose.covariance) == 36 * sizeof(double));
#endif

  memcpy(&(altOdomMsg.pose.covariance), pose_cov,
         sizeof(altOdomMsg.pose.covariance));
  memcpy(&(altOdomMsg.twist.covariance), twist_cov,
         sizeof(altOdomMsg.twist.covariance));

#ifdef DEBUG_
  ROS_INFO("Altered msg. new_msg pos cov[%d]=%f", i + 1,
           altOdomMsg.pose.covariance[i]);
#endif

  return altOdomMsg;
}

void MsgTransformer::subscribeAndPublish(const nav_msgs::Odometry &odom_msg) {
  nav_msgs::Odometry newMsg = alterMsg(odom_msg); // Alter the message

#ifdef DEBUG_
  int j = 18;
  ROS_INFO("Before pub pos cov[%d]=%f", j + 1, newMsg.pose.covariance[j]);
#endif

  pub_.publish(newMsg);
}

int main(int argc, char **argv) {
  // Initiate node and set handle
  ros::init(argc, argv, "msg_transformer_odom");
  ros::NodeHandle nh;

  ROS_INFO("Initiated msg_transformer node");

  // Create MsgTransformer object
  std::string subTopic;
  int pubQueueSize, subQueueSize;
  std::vector<double> poseCovariance;
  std::vector<double> twistCovariance;
  nh.getParam("subscriber_topic", subTopic);
#ifdef DEBUG_
  ROS_INFO("Got subscribe topic %s", subTopic.c_str());
#endif
  nh.getParam("pub_queue_size", pubQueueSize);
#ifdef DEBUG_
  ROS_INFO("Got pubQueueSize %i", pubQueueSize);
#endif
  nh.getParam("sub_queue_size", subQueueSize);
  nh.getParam("pose_cov", poseCovariance);
#ifdef DEBUG_
  ROS_INFO("Got poseCovariance[0]= %f", poseCovariance.front());
#endif
  nh.getParam("twist_cov", twistCovariance);

  MsgTransformer mt = MsgTransformer(nh, subTopic, pubQueueSize, subQueueSize,
                                     poseCovariance, twistCovariance);
  ROS_INFO("Created msg_transformer object");

  // Start subscribing and publishing to alternate
  while (ros::ok()) { // Alternate: ros::Rate(_), rate.sleep();
    ros::spinOnce();
  } // The above is equivalent to ros::spin();
    // Exit code here
}