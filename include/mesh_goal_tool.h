

#ifndef RVIZ_MESH_PLUGIN__MESH_GOAL_TOOL_H_
#define RVIZ_MESH_PLUGIN__MESH_GOAL_TOOL_H_

#include "mesh_pose_tool.h"
#include <geometry_msgs/PoseStamped.h>
#include <rviz/properties/string_property.h>
#include <rviz/display_context.h>


#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>
#endif

namespace rviz_mesh_plugin{
  class MeshGoalTool : public MeshPoseTool{
	 Q_OBJECT
	 public:
	   MeshGoalTool();
	 
	   virtual void onInitialize();
	 
	 private Q_SLOTS:
	   void updateTopic();
	 
	 protected:
	   virtual void onPoseSet( const Ogre::Vector3& position, const Ogre::Quaternion& orientation );
	   
	   rviz::StringProperty* topic_property_;
	   ros::Publisher pose_pub_;
	   ros::NodeHandle nh_;
  };

} /* namespace rviz_mesh_plugin */

#endif /* mesh_goal_tool.h */
