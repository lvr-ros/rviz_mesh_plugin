

#ifndef RVIZ_MESH_PLUGIN__MESH_POSE_TOOL_H_
#define RVIZ_MESH_PLUGIN__MESH_POSE_TOOL_H_

 #include <OGRE/OgreVector3.h>
 #include <OGRE/OgreQuaternion.h>
 #include <OGRE/OgreManualObject.h>
 #include <OGRE/OgreRay.h>
 
 #include <QCursor>
 #include <ros/ros.h>
 #include <rviz/tool.h>
 #include <rviz/ogre_helpers/arrow.h>
 
namespace rviz_mesh_plugin{
 
class MeshPoseTool: public rviz::Tool{
 public:
   MeshPoseTool();
   virtual ~MeshPoseTool();
 
   virtual void onInitialize();
 
   virtual void activate();
   virtual void deactivate();
 
   virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
 
 protected:
   virtual void onPoseSet( const Ogre::Vector3& position, const Ogre::Quaternion& orientation ) = 0;
 
   void getRawManualObjectData(
     const Ogre::ManualObject *mesh,
     const size_t sectionNumber,
     size_t& vertexCount,
     Ogre::Vector3*& vertices,
     size_t& indexCount,
     unsigned long*& indices);
     
   bool getPositionAndOrientation(
     const Ogre::ManualObject* mesh,
     const Ogre::Ray &ray,
     Ogre::Vector3& position,
     Ogre::Vector3& orientation);
     
   bool selectTriangle(
     rviz::ViewportMouseEvent& event,
     Ogre::Vector3& position,
     Ogre::Vector3& orientation);

   rviz::Arrow* arrow_;
   enum State
   {
     Position,
     Orientation
   };
   State state_;
   Ogre::Vector3 pos_;
   Ogre::Vector3 ori_;
};

} /* namespace rviz_mesh_plugin */

#endif /* mesh_pose_tool.h */
