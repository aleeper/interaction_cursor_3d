/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_INTERACTION_CURSOR_DISPLAY_H
#define RVIZ_INTERACTION_CURSOR_DISPLAY_H

#include "interaction_cursor_msgs/InteractionCursorUpdate.h"
#include "rviz/display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/bit_allocator.h>


namespace rviz
{

class Shape;
class FloatProperty;
class TfFrameProperty;
class RosTopicProperty;

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

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void updateCallback(const interaction_cursor_msgs::InteractionCursorUpdateConstPtr &icu_cptr);

  void getIntersections(const Ogre::Sphere &sphere);

private Q_SLOTS:
  /** @brief Update the length and radius of the axes object from property values. */
  void updateShape();

private:
  Shape* axes_;      ///< Handles actually drawing the axes

  FloatProperty* length_property_;
  FloatProperty* radius_property_;
  TfFrameProperty* frame_property_;
  RosTopicProperty* update_topic_property_;

  ros::Subscriber subscriber_update_;
  ros::Publisher publisher_feedback_;

};

} // namespace rviz

 #endif


