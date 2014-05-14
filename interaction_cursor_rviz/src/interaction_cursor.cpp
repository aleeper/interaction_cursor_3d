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

#include "interaction_cursor_rviz/interaction_cursor.h"
#include "interaction_cursor_msgs/InteractionCursorUpdate.h"
#include "interaction_cursor_msgs/InteractionCursorFeedback.h"


#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/view_manager.h"
#include "rviz/msg_conversions.h"

#include "rviz/selection/forwards.h"
#include "rviz/selection/selection_handler.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/default_plugin/interactive_markers/interactive_marker.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRenderable.h>
#include <OGRE/OgreRenderWindow.h>

#include <QApplication>
#include <QMenu>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

using namespace interaction_cursor_msgs;

namespace rviz
{

/** This class is needed to process scene query results. */
class MySceneQueryListener : public Ogre::SceneQueryListener
{
public:
  MySceneQueryListener() : disp_(0) { }
  virtual ~MySceneQueryListener() { }
    /** Called when a MovableObject is returned by a query.
    @remarks
        The implementor should return 'true' to continue returning objects,
        or 'false' to abandon any further results from this query.
    */
  virtual bool queryResult(Ogre::MovableObject* object)
  {
    // Do a simple attempt to refine the search by checking the "local" oriented bounding box:
    const Ogre::AxisAlignedBox& local_bb = object->getBoundingBox();
    Ogre::Sphere local_query_sphere = query_sphere_;
    local_query_sphere.setCenter(object->getParentNode()->convertWorldToLocalPosition(local_query_sphere.getCenter()));
    local_query_sphere.setRadius(local_query_sphere.getRadius()*2); // magic number... not sure why this is needed
    // If the sphere doesn't at least intersect the local bounding box, we skip this result.
    if(!local_bb.intersects(local_query_sphere)) return true;


    // If we got to here, we are assuming that we have satisfactory intersection.
    // TODO it would be better if we could keep a list of all intersecting objects, and then keep the smallest
    // one or something...
    //ROS_INFO("Sphere collides with MoveableObject [%s].",  object->getName().c_str());

    // The new rviz change adds a user object binding called "pick_handle" to objects when the pick color is set.
    Ogre::Any handle_any = object->getUserObjectBindings().getUserAny( "pick_handle" );

    if( handle_any.isEmpty() )
    {
      return true;
    }
    // Get the CollObjectHandle
    CollObjectHandle handle = Ogre::any_cast<CollObjectHandle>(handle_any);

    rviz::SelectionHandler* handler = disp_->getDisplayContext()->getSelectionManager()->getHandler(handle);
    if(handle)
    {
      InteractiveObjectWPtr ptr = handler->getInteractiveObject();

      // Don't do anything to a control if we are already grabbing its parent marker.
      if(ptr.lock() == disp_->grabbed_object_.lock())
        return true;

      //weak_ptr<Fruit> fruit = weak_ptr<Fruit>(dynamic_pointer_cast<Fruit>(food.lock());
      boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
      if(control && control->getVisible())
      {
        control->setHighlight(InteractiveMarkerControl::HOVER_HIGHLIGHT);
        disp_->highlighted_objects_.insert(ptr);
        //disp_->grabbed_object_ = ptr;

        // returning false means no subsequent object will be checked.
        return false;
      }
    }
    return true;
  }

  /** Called when a WorldFragment is returned by a query.
  @remarks
      The implementor should return 'true' to continue returning objects,
      or 'false' to abandon any further results from this query.
  */
  virtual bool queryResult(Ogre::SceneQuery::WorldFragment* fragment)
  {
    //ROS_INFO("Sphere collides with WorldFragment type [%d].", fragment->fragmentType );
    return true;
  }
  rviz::InteractionCursorDisplay* disp_;
  Ogre::Sphere query_sphere_;
};

// -----------------------------------------------------------------------------


InteractionCursorDisplay::InteractionCursorDisplay()
  : Display()
  , nh_("")
  , cursor_shape_(0)
  , dragging_(false)
  , current_menu_(0)
  , current_submenu_(0)
{
  grabbed_object_.reset();

  update_topic_property_ = new RosTopicProperty( "Update Topic", "/interaction_cursor/update",
                                                        ros::message_traits::datatype<interaction_cursor_msgs::InteractionCursorUpdate>(),
                                                        "interaction_cursor_msgs::InteractionCursorUpdate topic to subscribe to.",
                                                        this, SLOT( changeUpdateTopic() ));

  show_cursor_shape_property_ = new BoolProperty("Show Cursor", true,
                                                 "Enables display of cursor shape.",
                                                 this, SLOT( updateShape() ));

  show_cursor_axes_property_ = new BoolProperty("Show Axes", true,
                                                "Enables display of cursor axes.",
                                                this, SLOT( updateAxes()));

  axes_length_property_ = new FloatProperty( "Axes Length", 0.1,
                                        "Length of each axis, in meters.",
                                        this, SLOT( updateAxes() ));
  axes_length_property_->setMin( 0.0001 );

  axes_radius_property_ = new FloatProperty( "Axes Radius", 0.01,
                                        "Radius of each axis, in meters.",
                                        this, SLOT( updateAxes() ));
  axes_radius_property_->setMin( 0.0001 );

  shape_scale_property_ = new FloatProperty( "Cursor Diameter", 0.1,
                                        "Size of cursor, in meters.",
                                        this, SLOT( updateShape() ));
  shape_scale_property_->setMin( 0.0001 );

  shape_color_property_ = new ColorProperty("Cursor Color", QColor(80, 255, 20),
                                            "Color of cursor.",
                                            this, SLOT(updateShape()));

  shape_alpha_property_ = new FloatProperty( "Cursor Alpha", 1.0,
                                             "Alpha value of cursor.",
                                             this, SLOT( updateShape()) );
  shape_alpha_property_->setMin(0.0f);
  shape_alpha_property_->setMax(1.0f);
}

InteractionCursorDisplay::~InteractionCursorDisplay()
{
  delete cursor_shape_;
  delete cursor_axes_;
  context_->getSceneManager()->destroySceneNode( cursor_node_ );
}

void InteractionCursorDisplay::changeUpdateTopic()
{
  std::string tmp = update_topic_property_->getStdString();

  subscriber_update_ = nh_.subscribe<interaction_cursor_msgs::InteractionCursorUpdate>
                              (tmp, 30,
                              boost::bind(&InteractionCursorDisplay::updateCallback, this, _1));
  tmp.replace(tmp.find("update"), tmp.length(), "feedback");
  publisher_feedback_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorFeedback>
                              (tmp, 30);
}

void InteractionCursorDisplay::onInitialize()
{
  cursor_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  cursor_node_->setVisible( isEnabled() );

  cursor_axes_ = new Axes( scene_manager_, cursor_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat() );
  cursor_axes_->getSceneNode()->setVisible( show_cursor_axes_property_->getBool() );

  cursor_shape_ = new Shape( Shape::Sphere, context_->getSceneManager(), cursor_node_);
  updateShape();

  // Should this happen onEnable and then get killed later?

  //changeFeedbackTopic();
}

void InteractionCursorDisplay::onEnable()
{
  cursor_node_->setVisible( true );
  changeUpdateTopic();
}

void InteractionCursorDisplay::onDisable()
{
  cursor_node_->setVisible( false, true );
  subscriber_update_.shutdown();
}

void InteractionCursorDisplay::updateAxes()
{
  cursor_axes_->set( axes_length_property_->getFloat(), axes_radius_property_->getFloat() );
  cursor_axes_->getSceneNode()->setVisible( show_cursor_axes_property_->getBool(), true);
  context_->queueRender();
}


void InteractionCursorDisplay::updateShape()
{
  Ogre::Vector3 shape_scale( 1.01*shape_scale_property_->getFloat());
  cursor_shape_->setScale( shape_scale );
  cursor_shape_->getRootNode()->setVisible( show_cursor_shape_property_->getBool(), true );
  Ogre::ColourValue color = shape_color_property_->getOgreColor();
  color.a = shape_alpha_property_->getFloat();
  cursor_shape_->setColor(color);
  context_->queueRender();
}

ViewportMouseEvent InteractionCursorDisplay::createMouseEvent(uint8_t button_state)
{
  ViewportMouseEvent event;
  event.panel = context_->getViewManager()->getRenderPanel();
  event.viewport = context_->getViewManager()->getRenderPanel()->getRenderWindow()->getViewport(0);

  if(button_state == interaction_cursor_msgs::InteractionCursorUpdate::NONE)
  {
    event.type = QEvent::None;
  }
  if(button_state == interaction_cursor_msgs::InteractionCursorUpdate::GRAB)
  {
    event.type = QEvent::MouseButtonPress;
    event.acting_button = Qt::LeftButton;
  }
  if(button_state == interaction_cursor_msgs::InteractionCursorUpdate::KEEP_ALIVE)
  {
    event.type = QEvent::MouseMove;
    event.buttons_down = event.buttons_down | Qt::LeftButton;
  }
  if(button_state == interaction_cursor_msgs::InteractionCursorUpdate::RELEASE)
  {
    event.type = QEvent::MouseButtonRelease;
    event.acting_button = Qt::LeftButton;
  }
  if(button_state == interaction_cursor_msgs::InteractionCursorUpdate::QUERY_MENU)
  {
    event.type = QEvent::MouseButtonRelease;
    event.acting_button = Qt::RightButton;
    event.buttons_down = Qt::NoButton;
  }
  return event;
}

bool InteractionCursorDisplay::generateKeyEvent(uint8_t key_event)
{
  QKeyEvent *event = 0;
  QKeyEvent *event2 = 0;

  switch(key_event)
  {
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_UP):
      ROS_DEBUG("Posting a KEY_UP event...");
      event =  new QKeyEvent(QEvent::KeyPress, Qt::Key_Up, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Up, Qt::NoModifier);
      break;
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_DOWN):
      ROS_DEBUG("Posting a KEY_DOWN event...");
      event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Down, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Down, Qt::NoModifier);
      break;
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_LEFT):
      ROS_DEBUG("Posting a KEY_LEFT event...");
      event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Left, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Left, Qt::NoModifier);
      break;
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_RIGHT):
      ROS_DEBUG("Posting a KEY_RIGHT event...");
      event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Right, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Right, Qt::NoModifier);
      break;
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_ENTER):
      ROS_DEBUG("Posting a KEY_ENTER event...");
      event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Enter, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Enter, Qt::NoModifier);
      break;
    case (interaction_cursor_msgs::InteractionCursorUpdate::KEY_ESCAPE):
      ROS_DEBUG("Posting a KEY_ESCAPE event...");
      event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
      event2 = new QKeyEvent(QEvent::KeyRelease, Qt::Key_Escape, Qt::NoModifier);
      break;
    default:
      break;
  }

  if (event && event2)
  {
    if(current_menu_ && current_menu_->isVisible())
    {
      QWidget* receiver = current_submenu_ ? current_submenu_ : current_menu_;

      if ( (event->key() == Qt::Key_Right || event->key() == Qt::Key_Enter)
           && current_menu_->activeAction() && current_menu_->activeAction()->menu())
      {
        current_submenu_ = current_menu_->activeAction()->menu();
      }
      else if ( event->key() == Qt::Key_Left )
      {
        current_submenu_ = 0;
      }
      else if ( event->key() == Qt::Key_Escape)
      {
        current_submenu_ = 0;
      }

      QApplication::postEvent(receiver, event);
      QApplication::postEvent(receiver, event2);
    }
    else
      current_menu_ = current_submenu_ = 0;

    return true;
  }

  return false;
}

void InteractionCursorDisplay::updateCallback(const interaction_cursor_msgs::InteractionCursorUpdateConstPtr &icu_cptr)
{
  if( !this->isEnabled() )
    return;

  std::string frame = icu_cptr->pose.header.frame_id;
  Ogre::Vector3 position;
  Ogre::Quaternion quaternion;

  if( context_->getFrameManager()->transform(frame, ros::Time(0), icu_cptr->pose.pose, position, quaternion) )
  {
    cursor_node_->setPosition( position );
    cursor_node_->setOrientation( quaternion );
    updateShape();

    Ogre::Sphere sphere(position, shape_scale_property_->getFloat()/2.0);
    clearOldSelections();

    if(icu_cptr->key_event != icu_cptr->NONE)  // Fake some keyboard events for menu navigation
    {
      getIntersections(sphere);
      generateKeyEvent(icu_cptr->key_event);
      return;
    }
    else if(icu_cptr->button_state == icu_cptr->NONE)
    {
      getIntersections(sphere);
      boost::shared_ptr<InteractiveMarkerControl> control;
      InteractiveObjectWPtr ptr;
      getActiveControl(ptr, control);
      // Does the right thing even if control is null
      sendInteractionFeedback(interaction_cursor_msgs::InteractionCursorFeedback::NONE,
                              control, position, quaternion);
    }
    else if(icu_cptr->button_state == icu_cptr->GRAB)
    {
      getIntersections(sphere);
      grabObject(position, quaternion, createMouseEvent(icu_cptr->button_state));
    }
    else if(icu_cptr->button_state == icu_cptr->KEEP_ALIVE)
    {
      updateGrabbedObject(position, quaternion, createMouseEvent(icu_cptr->button_state));
    }
    else if(icu_cptr->button_state == icu_cptr->RELEASE)
    {
      releaseObject(position, quaternion, createMouseEvent(icu_cptr->button_state));
    }
    else if(icu_cptr->button_state == icu_cptr->QUERY_MENU)
    {
      getIntersections(sphere);
      requestMenu(position, quaternion, createMouseEvent(icu_cptr->button_state));
    }
    context_->queueRender();

    setStatus( StatusProperty::Ok, "Transform", "Transform OK" );
  }
  else
  {
    std::string error;
    if( context_->getFrameManager()->transformHasProblems( frame, ros::Time(), error ))
    {
      setStatus( StatusProperty::Error, "Transform", QString::fromStdString( error ));
    }
    else
    {
      setStatus( StatusProperty::Error,
                 "Transform",
                 "Could not transform from [" + QString::fromStdString(frame) + "] to Fixed Frame [" + fixed_frame_ + "] for an unknown reason" );
    }
  }
}

void InteractionCursorDisplay::clearOldSelections()
{
  std::set<InteractiveObjectWPtr>::iterator it;
  for ( it=highlighted_objects_.begin() ; it != highlighted_objects_.end(); it++ )
  {
    InteractiveObjectWPtr ptr = (*it);
    if(!ptr.expired())
    {
      boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
      if(control)
      {
        control->setHighlight(InteractiveMarkerControl::NO_HIGHLIGHT);
        //disp_->highlighted_objects.insert(ptr);
      }
    }
  }
  highlighted_objects_.clear();
}

void InteractionCursorDisplay::getIntersections(const Ogre::Sphere &sphere)
{
  Ogre::SphereSceneQuery* ssq = context_->getSceneManager()->createSphereQuery(sphere);
  MySceneQueryListener listener;
  listener.disp_ = this;
  listener.query_sphere_ = sphere;
  ssq->execute(&listener);
  context_->getSceneManager()->destroyQuery(ssq);
}

void InteractionCursorDisplay::sendInteractionFeedback(uint8_t event_type,
                                                       const boost::shared_ptr<InteractiveMarkerControl>& control,
                                                       const Ogre::Vector3& cursor_pos,
                                                       const Ogre::Quaternion& cursor_rot)
{
  std::string code_string = "control_frame: ";
  std::string name = "", description = "", frame = "";
  int interaction_mode = 0;
  if(!control)
  {
    //ROS_INFO("No control detected.");
    interaction_cursor_msgs::InteractionCursorFeedback fb;
    fb.event_type = event_type;
    // empty string means no interactive marker.
    fb.pose.header.frame_id = "";
    publisher_feedback_.publish(fb);
    return;
  }
  else //control
  {
    interaction_mode = control->getInteractionMode();
    name = control->getName();
    description = control->getDescription().toStdString();
    if(description == "")
    {
      frame = "no_frame";
    }
    else
    {
      frame = description;
      if(frame.find(code_string) != std::string::npos)
      {
        frame.replace(0, code_string.length(), "" );
      }
      else
      {
        frame = "no_frame";
      }
    }
  }

  if( (frame != "" && frame != "no_frame")
     && (event_type == interaction_cursor_msgs::InteractionCursorFeedback::NONE || event_type == interaction_cursor_msgs::InteractionCursorFeedback::GRABBED ))
  {

    // Extract frame and compute pose for the grabbed marker
    interaction_cursor_msgs::InteractionCursorFeedback fb;
    fb.event_type = event_type;
    if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_AXIS
       || interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_PLANE
       || interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS)
    {
      fb.attachment_type = fb.POSITION;
    }
    else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE)
    {
      fb.attachment_type = fb.POSITION_AND_ORIENTATION;
    }

    Ogre::Vector3 pos_world_to_control;
    Ogre::Quaternion rot_world_to_control;
    if( context_->getFrameManager()->getTransform(frame, ros::Time(0), pos_world_to_control, rot_world_to_control))
    {
      Ogre::Vector3 pos_control_to_cursor_in_control_frame = rot_world_to_control.Inverse()*(cursor_pos - pos_world_to_control);
      Ogre::Quaternion rot_control_to_cursor = rot_world_to_control.Inverse()*cursor_rot;
      pointOgreToMsg(pos_control_to_cursor_in_control_frame, fb.pose.pose.position);
      quaternionOgreToMsg(rot_control_to_cursor, fb.pose.pose.orientation);
      fb.pose.header.frame_id = frame;
      fb.pose.header.stamp = ros::Time(0);
      publisher_feedback_.publish(fb);
    }
    else
    {
      std::string error;
      if( context_->getFrameManager()->transformHasProblems( frame, ros::Time(), error ))
      {
        ROS_ERROR_STREAM( error );
      }
      else
      {
        ROS_ERROR_STREAM("Could not transform from [" + frame + "] to Fixed Frame [" + fixed_frame_.toStdString() + "] for an unknown reason" );
      }
      fb.pose.header.frame_id = "no_frame";
      fb.attachment_type = fb.NONE;
      publisher_feedback_.publish(fb);
    }
  }
  else // No control frame detected
  {
    //ROS_INFO("No control frame detected for control [%s] with description [%s]", name.c_str(), description.c_str());
    interaction_cursor_msgs::InteractionCursorFeedback fb;
    fb.event_type = event_type;
    // empty string means no marker; "no_frame" means there is a control to frag, but no associated control frame.
    fb.pose.header.frame_id = frame;
    publisher_feedback_.publish(fb);
  }
}

void InteractionCursorDisplay::getActiveControl(InteractiveObjectWPtr& ptr, boost::shared_ptr<InteractiveMarkerControl>& control)
{
  if(!grabbed_object_.expired())
  {
    ptr = grabbed_object_;
    //highlighted_objects_.erase(grabbed_object_);
  }
  else if(highlighted_objects_.begin() == highlighted_objects_.end())
    return;
  else
  {
    getBestControl(ptr);
    // Remove the object from the set so that we don't un-highlight it later on accident.
    //highlighted_objects_.erase(highlighted_objects_.begin());
  }


  if(!ptr.expired())
  {
    control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
  }
}

// Choose the 'best' ptr/control from the set of highlighted objects,
// where 'best' = most degrees of freedom.
void InteractionCursorDisplay::getBestControl(InteractiveObjectWPtr& ptr)
{
  ptr = *(highlighted_objects_.begin());
  boost::shared_ptr<InteractiveMarkerControl> control;
  control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());

  BOOST_FOREACH(InteractiveObjectWPtr candidate_ptr, highlighted_objects_) {
    boost::shared_ptr<InteractiveMarkerControl> candidate_control;
    candidate_control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(candidate_ptr.lock());
    if (candidate_control->getInteractionMode() >= control->getInteractionMode())
    {
      ptr = candidate_ptr;
      control = candidate_control;
    }
  }
}

void InteractionCursorDisplay::grabObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event)
{
  boost::shared_ptr<InteractiveMarkerControl> control;
  InteractiveObjectWPtr ptr;
  getActiveControl(ptr, control);
  if(control)
  {
    ROS_DEBUG("Grabbing object [%s]", control->getName().c_str());
    control->handle3DCursorEvent(event, position, orientation);
    sendInteractionFeedback(interaction_cursor_msgs::InteractionCursorFeedback::GRABBED,
                            control, position, orientation);
    grabbed_object_ = ptr;
    highlighted_objects_.erase(grabbed_object_);
    dragging_ = true;
  }
}

void InteractionCursorDisplay::updateGrabbedObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event)
{
  boost::shared_ptr<InteractiveMarkerControl> control;
  InteractiveObjectWPtr ptr;
  getActiveControl(ptr, control);
  if(dragging_ && control)
  {
    control->handle3DCursorEvent(event, position, orientation);
    sendInteractionFeedback(interaction_cursor_msgs::InteractionCursorFeedback::KEEP_ALIVE,
                            control, position, orientation);
  }
  else if(dragging_)
  {
    ROS_WARN("Grabbed object weak pointer seems to have expired...");
    sendInteractionFeedback(interaction_cursor_msgs::InteractionCursorFeedback::LOST_GRASP,
                            boost::shared_ptr<InteractiveMarkerControl>(), position, orientation);
    grabbed_object_.reset();
    dragging_ = false;
  }
}

void InteractionCursorDisplay::releaseObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event)
{
  boost::shared_ptr<InteractiveMarkerControl> control;
  InteractiveObjectWPtr ptr;
  getActiveControl(ptr, control);
  if(dragging_ && control)
  {
    ROS_DEBUG("Releasing object [%s]", control->getName().c_str());
    control->handle3DCursorEvent(event, position, orientation);
    // Add it back to the set for later un-highlighting.
    highlighted_objects_.insert(grabbed_object_);
  }
  else if( dragging_ )
  {
    ROS_WARN("Grabbed object seems to have expired before we released it!");
  }
  sendInteractionFeedback(interaction_cursor_msgs::InteractionCursorFeedback::RELEASED,
                          control, position, orientation);
  grabbed_object_.reset();
  dragging_ = false;
}

void InteractionCursorDisplay::requestMenu(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation, const rviz::ViewportMouseEvent &event)
{
  ROS_DEBUG("Requesting a menu");
  if(highlighted_objects_.begin() == highlighted_objects_.end())
    return;
  InteractiveObjectWPtr ptr = *(highlighted_objects_.begin());
  if(!ptr.expired())
  {
    boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
    if(control)
    {
      control->handle3DCursorEvent(event, position, orientation);
      current_menu_ = control->getParent()->getMenu().get();
      current_submenu_ = 0;
    }
  }
  else
  {
    current_menu_ = current_submenu_ = 0;
  }
}

//void InteractionCursorDisplay::

void InteractionCursorDisplay::update( float dt, float ros_dt )
{

}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
// namespace, name, type, base type
PLUGINLIB_EXPORT_CLASS( rviz::InteractionCursorDisplay, rviz::Display )



