

#include "mesh_goal_tool.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_mesh_plugin::MeshGoalTool, rviz::Tool )

namespace rviz_mesh_plugin{
MeshGoalTool::MeshGoalTool()
{
   shortcut_key_ = 'm'; 
     topic_property_ = new rviz::StringProperty( "Topic", "goal",
     "The topic on which to publish the mesh navigation goals.",
     getPropertyContainer(), SLOT( updateTopic() ), this );
	 //pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
 }
 
 void MeshGoalTool::onInitialize()
 {
   MeshPoseTool::onInitialize();
   setName( "Mesh Goal" );
   updateTopic();
 }
 
 void MeshGoalTool::updateTopic()
 {
   pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
 }
  
 void MeshGoalTool::onPoseSet( const Ogre::Vector3& position, const Ogre::Quaternion& orientation ){
  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = position.x;
  msg.pose.position.y = position.y;
  msg.pose.position.z = position.z;
  
  // ogreToRos(x,y,z) = (-z,-x,y) 
  Ogre::Quaternion ros_orientation( 
	-orientation.zAxis(),
	-orientation.xAxis(),
	orientation.yAxis()
  );	  
  
  msg.pose.orientation.x = ros_orientation.x;
  msg.pose.orientation.y = ros_orientation.y;
  msg.pose.orientation.z = ros_orientation.z;
  msg.pose.orientation.w = ros_orientation.w;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  pose_pub_.publish(msg);
 }
	
}
