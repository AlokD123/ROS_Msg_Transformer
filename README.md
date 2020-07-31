# README
Package for a custom ROS message transformer - i.e. a node that subscribes to a topic, alters incoming message, and then publishes them to another topic.

Motivation: to modify the outputs from existing Gazebo simulation plugins, allowing for simulated sensor values to be partially altered to match more realistic values in "messages" on-the-fly.

For more information about ROS and message, see: http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes

Status: the "MessageTransformer" class is currently limited to GPS sensor messages. This will be extended to more messages in the future.
