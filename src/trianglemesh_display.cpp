/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  trianglemesh_display.cpp
 *
 *
 *  authors: 
 *    
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Henning Deeken <hdeeken@uni-osnabrueck.de>
 *    Marcel Mrozinski
 *    Nils Oesting
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>

#include "trianglemesh_display.h"

#include <ros/callback_queue.h>

namespace rviz_mesh_plugin
{

  size_t TriangleMeshDisplay::displayCounter = 0;

  TriangleMeshDisplay::TriangleMeshDisplay()
  {
    m_displayID = displayCounter++;
    m_tfMeshFilter = 0;
    m_meshCounter = 0;

    // topic properties
    m_meshTopic = 
      new rviz::RosTopicProperty(
          "Topic",
          "",
          QString::fromStdString(ros::message_traits::datatype<mesh_msgs::TriangleMeshStamped>()),
          "Mesh topic to subscribe to.",
          this,
          SLOT(updateTopic())
          );

    // mesh buffer size property
    m_meshBufferSize = 
      new rviz::IntProperty(
          "Mesh Buffer Size",
          1,
          "Number of prior meshes to display.",
          this,
          SLOT(updateMeshBufferSize())
          );
    m_meshBufferSize->setMin(1);

    // wireframe properties
    m_showWireframe =
      new rviz::BoolProperty(
          "Show Wireframe",
          true,
          "Show or hide the wireframe of the mesh.",
          this,
          SLOT(updateMesh())
          );
    // wireframe color property
    m_wireframeColor = 
      new rviz::ColorProperty(
          "Color",
          QColor(0, 0, 0),
          "The color of the wireframe.",
          m_showWireframe,
          SLOT(updateMesh()),
          this
          );
    // wireframe alpha property
    m_wireframeAlpha = 
      new rviz::FloatProperty(
          "Alpha",
          1.0,
          "The alpha-value of the wireframe",
          m_showWireframe,
          SLOT(updateMesh()),
          this
          );
    m_wireframeAlpha->setMin(0);
    m_wireframeAlpha->setMax(1);

    // face properties
    m_showFaces = 
      new rviz::BoolProperty(
          "Show Faces",
          true,
          "Show or hide the faces of the mesh.",
          this,
          SLOT(updateMesh())
          );
    // face color properties
    m_facesColor =
      new rviz::ColorProperty(
          "Color",
          QColor(0, 255, 0),
          "The color of the faces.",
          m_showFaces,
          SLOT(updateMesh()),
          this
          );

    // face alpha properties
    m_facesAlpha =
      new rviz::FloatProperty(
          "Alpha",
          1.0,
          "The alpha-value of the faces",
          m_showFaces,
          SLOT(updateMesh()),
          this
          );
    m_facesAlpha->setMin(0);
    m_facesAlpha->setMax(1);

    // face vertex color propertiy
    m_facesVertexColors = 
      new rviz::BoolProperty(
          "Use Vertex Color",
          false,
          "Use the vertex color for the faces, if enabled and vertex colors are available",
          m_showFaces,
          SLOT(updateMesh()),
          this
          );
    
    // face color propertiy
    m_facesTriangleColors = 
      new rviz::BoolProperty(
          "Use Triangle Color",
          false,
          "Use the triangle color for the faces, if enabled and triangle colors are available",
          m_showFaces,
          SLOT(updateMesh()),
          this
          );


    // Normal propertys
    m_showNormals = 
      new rviz::BoolProperty("Show Normals",
          true,
          "Show or hide the normals of the mesh.",
          this,
          SLOT(updateMesh())
          );
    m_normalsColor =
      new rviz::ColorProperty("Color",
          QColor(255, 0, 255),
          "The color of the normals.",
          m_showNormals,
          SLOT(updateMesh()),
          this
          );
    m_normalsAlpha = 
      new rviz::FloatProperty("Alpha",
          1.0,
          "The alpha-value of the normals",
          m_showNormals,
          SLOT(updateMesh()),
          this
          );
    m_normalsAlpha->setMin(0);
    m_normalsAlpha->setMax(1);
    m_scallingFactor = 
      new rviz::FloatProperty("Scalling Factor",
          0.1,
          "Scalling factor of the normals",
          m_showNormals,
          SLOT(updateMesh()),
          this
          );
    m_scallingFactor->setMin(0);

    // Texture propertys
    m_showTextures = 
      new rviz::BoolProperty("Show Textures",
          true,
          "Show or hide the textures of the mesh.",
          this,
          SLOT(updateMesh())
          );
  }

  TriangleMeshDisplay::~TriangleMeshDisplay()
  {
    unsubscribe();
    delete m_tfMeshFilter;
  }

  void TriangleMeshDisplay::onInitialize()
  {
    m_tfMeshFilter = new tf::MessageFilter<mesh_msgs::TriangleMeshStamped>(
        *rviz::Display::context_->getTFClient(),
        rviz::Display::fixed_frame_.toStdString(),
        1,
        rviz::Display::update_nh_
      );

    m_tfMeshFilter->connectInput(m_meshSubscriber);

    context_->getFrameManager()->registerFilterForTransformStatusCheck(m_tfMeshFilter, this);

    m_synchronizer = 0;

    updateMeshBufferSize();
    updateTopic();
    updateMesh();
  }

  void TriangleMeshDisplay::reset()
  {
    rviz::Display::reset(); // TODO bad hack?!
    m_tfMeshFilter->clear();
    m_messagesReceived = 0;
    m_meshVisuals.clear();
  }

  void TriangleMeshDisplay::updateTopic()
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  void TriangleMeshDisplay::subscribe()
  {
    if (!isEnabled()){
      return;
    }

    try{
      m_meshSubscriber.subscribe(update_nh_, m_meshTopic->getTopicStd(), 1);
      setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
    }catch(ros::Exception& e){
      setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }

    // Nothing
    if (m_meshTopic->getTopicStd().empty())
    {
      m_meshBufferSize->hide();
      m_showWireframe->hide();
      m_showWireframe->setBool(false);

      m_showFaces->hide();
      m_showFaces->setBool(false);

      m_showTextures->hide();
      m_showTextures->setBool(false);

      m_showNormals->hide();
      m_showNormals->setBool(false);
      return;
    }

    else
    {
      // Geometry & Texture & PointNormals
      m_meshBufferSize->show();
      m_showWireframe->show();
      m_showWireframe->setBool(true);
      m_showFaces->show();
      m_showFaces->setBool(true);
      m_showTextures->show();
      m_showTextures->setBool(true);
      m_showNormals->show();
      m_showNormals->setBool(true);

      m_synchronizer = 
        new message_filters::Cache<mesh_msgs::TriangleMeshStamped>(
          m_meshSubscriber, 10
        );

      m_synchronizer->registerCallback(
        boost::bind(&TriangleMeshDisplay::incomingMessage, this, _1)
      );
    }
  }

  void TriangleMeshDisplay::unsubscribe()
  {
    m_meshSubscriber.unsubscribe();

    if (m_synchronizer)
    {
      delete m_synchronizer;
      m_synchronizer = 0;
    }
  }

  void TriangleMeshDisplay::onEnable()
  {
    subscribe();
  }

  void TriangleMeshDisplay::onDisable()
  {
    unsubscribe();
    reset();
  }

  void TriangleMeshDisplay::fixedFrameChanged()
  {
    m_tfMeshFilter->setTargetFrame(fixed_frame_.toStdString());
    reset();
  }

  void TriangleMeshDisplay::incomingMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg)
  {
    m_messagesReceived++;
    setStatus(rviz::StatusProperty::Ok, "Topic", QString::number(m_messagesReceived) + " messages received");
    processMessage(meshMsg);
  }

  void TriangleMeshDisplay::updateMeshBufferSize()
  {
    m_meshVisuals.rset_capacity(m_meshBufferSize->getInt());
  }

  void TriangleMeshDisplay::updateMesh()
  {
    if (!m_meshVisuals.empty())
    {
      for (boost::circular_buffer<boost::shared_ptr<TriangleMeshVisual> >::iterator it = m_meshVisuals.begin();
          it != m_meshVisuals.end();
          it++)
      {
        it->get()->updateMaterial(m_showWireframe->getBool(), m_wireframeColor->getOgreColor(), m_wireframeAlpha->getFloat(),
            m_showFaces->getBool(), m_facesColor->getOgreColor(), m_facesAlpha->getFloat(), m_facesVertexColors->getBool(), m_facesTriangleColors->getBool(),
            m_showTextures->getBool(),
            m_showNormals->getBool(), m_normalsColor->getOgreColor(), m_normalsAlpha->getFloat(),
            m_scallingFactor->getFloat());
      }
    }
  }

  void TriangleMeshDisplay::processMessage(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg){
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if(!context_->getFrameManager()->getTransform(meshMsg->header.frame_id,
          meshMsg->header.stamp,
          position, orientation)
    ){
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
          meshMsg->header.frame_id.c_str(), qPrintable(rviz::Display::fixed_frame_));
      return;
    }

    boost::shared_ptr<TriangleMeshVisual> visual;
    if (m_meshVisuals.full()){
      visual = m_meshVisuals.front();
    }else{
      visual.reset(new TriangleMeshVisual(context_, m_displayID, m_meshCounter));
      m_meshVisuals.push_front(visual);
      m_meshCounter++;
    }
    visual->setMessage(meshMsg);
    updateMesh();
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);
  }
} // end namespace rviz_mesh_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_mesh_plugin::TriangleMeshDisplay, rviz::Display)
