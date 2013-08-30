/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Adam Leeper */

#ifndef RVIZ_INTERACTION_CURSOR_DISPLAY_H
#define RVIZ_INTERACTION_CURSOR_DISPLAY_H

#include "interaction_cursor_msgs/InteractionCursorUpdate.h"

#include <rviz/bit_allocator.h>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <QMenu>
#include <QAction>


namespace rviz
{

class Shape;
class Axes;
class BoolProperty;
class FloatProperty;
class TfFrameProperty;
class RosTopicProperty;
class ColorProperty;
class MyVisitor;
class MySceneQueryListener;

/** @brief Creates a 3D cursor for interaction. */
class InteractionCursorDisplay: public Display
{
Q_OBJECT
public:
  InteractionCursorDisplay();
  virtual ~InteractionCursorDisplay();

  void onInitialize();

  /**
   * \brief Set the parameters for the axes
   * @param length Length of each axis
   * @param radius Radius of each axis
   */
  void set( float length, float radius );

  // Overrides from Display
  virtual void update(float dt, float ros_dt);

  rviz::DisplayContext* getDisplayContext() { return context_; }

  friend class MyVisitor;
  friend class MySceneQueryListener;


protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  // This is the main callback function that processes new interaction cursor messages.
  void updateCallback(const interaction_cursor_msgs::InteractionCursorUpdateConstPtr &icu_cptr);

  void getIntersections(const Ogre::Sphere &sphere);

  void clearOldSelections();

  rviz::ViewportMouseEvent createMouseEvent(uint8_t button_state);

  // Return true if key event was posted
  bool generateKeyEvent(uint8_t key_event);

  void sendInteractionFeedback(uint8_t event_type,
                               const boost::shared_ptr<InteractiveMarkerControl>& control,
                               const Ogre::Vector3& cursor_pos,
                               const Ogre::Quaternion& cursor_rot);

  void getActiveControl(InteractiveObjectWPtr& ptr, boost::shared_ptr<InteractiveMarkerControl> & control);

  void getBestControl(InteractiveObjectWPtr& ptr);

  void grabObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event);
  void updateGrabbedObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event);
  void releaseObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event);
  void requestMenu(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event);

protected Q_SLOTS:
  /** @brief Update the length and radius of the axes object from property values. */
  void updateAxes();

  /** @brief Update the scale of the shape object from property values. */
  void updateShape();

  /** @brief Update the topic used to subscribe to updates. */
  virtual void changeUpdateTopic();

protected:

  ros::NodeHandle nh_;

  Ogre::SceneNode* cursor_node_;

  Shape* cursor_shape_;      ///< Handles actually drawing the axes
  Axes* cursor_axes_;

  BoolProperty*  show_cursor_axes_property_;
  BoolProperty*  show_cursor_shape_property_;
  FloatProperty* axes_length_property_;
  FloatProperty* axes_radius_property_;
  FloatProperty* shape_scale_property_;
  ColorProperty* shape_color_property_;
  FloatProperty* shape_alpha_property_;
  //TfFrameProperty* frame_property_;
  RosTopicProperty* update_topic_property_;
  RosTopicProperty* feedback_topic_property_;

  ros::Subscriber subscriber_update_;
  ros::Publisher publisher_feedback_;

  std::set<InteractiveObjectWPtr> highlighted_objects_;

  InteractiveObjectWPtr grabbed_object_;
  bool dragging_;

  Ogre::Vector3 position_offset_at_grab_;
  Ogre::Quaternion orientation_offset_at_grab_;
  std::string marker_frame_at_grab_;
  QMenu* current_menu_;
  QMenu* current_submenu_;

};

} // namespace rviz

 #endif


