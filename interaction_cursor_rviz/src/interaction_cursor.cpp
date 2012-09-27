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

#include "interaction_cursor_rviz/interaction_cursor.h"


#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/ros_topic_property.h"

#include "rviz/selection/forwards.h"
#include "rviz/selection/selection_handler.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/default_plugin/interactive_markers/interactive_marker.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRenderable.h>

#include <boost/bind.hpp>



namespace rviz
{

// Some convenience functions for Ogre / geometry_msgs conversions
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Point &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Vector3 &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline geometry_msgs::Point pointOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void pointOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Point &m)  { m.x = o.x; m.y = o.y; m.z = o.z; }

static inline geometry_msgs::Vector3 vectorOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void vectorOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Vector3 &m) { m.x = o.x; m.y = o.y; m.z = o.z; }

// -----------------------------------------------------------------------------
/** Visitor object that can be used to iterate over a collection of Renderable
instances abstractly.
@remarks
Different scene objects use Renderable differently; some will have a
single Renderable, others will have many. This visitor interface allows
classes using Renderable to expose a clean way for external code to
get access to the contained Renderable instance(s) that it will
eventually add to the render queue.
@par
To actually have this method called, you have to call a method on the
class containing the Renderable instances. One example is
MovableObject::visitRenderables.
*/
class MyVisitor : public Ogre::Renderable::Visitor
{
public:
  MyVisitor()
    : disp_(0) {}

  /** Virtual destructor needed as class has virtual methods. */
  virtual ~MyVisitor() { }
  /** Generic visitor method.
  @param rend The Renderable instance being visited
  @param lodIndex The LOD index to which this Renderable belongs. Some
    objects support LOD and this will tell you whether the Renderable
    you're looking at is from the top LOD (0) or otherwise
  @param isDebug Whether this is a debug renderable or not.
  @param pAny Optional pointer to some additional data that the class
    calling the visitor may populate if it chooses to.
  */
  virtual void visit(Ogre::Renderable* rend, ushort lodIndex, bool isDebug, Ogre::Any* pAny = 0)
  {
    // This only exists in Ogre version
    //if( !rend->hasCustomParameter(PICK_COLOR_PARAMETER) ) return;
    rviz::SelectionManager* sm = disp_->getDisplayContext()->getSelectionManager();

    try
    {
      Ogre::Vector4 vec = rend->getCustomParameter(PICK_COLOR_PARAMETER);
      if(sm)
      {
        Ogre::ColourValue colour(vec.x, vec.y, vec.z, 1.0);
        CollObjectHandle handle = sm->colourToHandle(colour);

        rviz::SelectionHandler* handler = sm->getHandler(handle);
        if(handle)
        {
          InteractiveObjectWPtr ptr = handler->getInteractiveObject();

          // Don't do anything to a control if we are already grabbing its parent marker.
          if(ptr.lock() == disp_->grabbed_object_.lock())
            return;

          //weak_ptr<Fruit> fruit = weak_ptr<Fruit>(dynamic_pointer_cast<Fruit>(food.lock());
          boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
          if(control)
          {
            control->setHoverHighlight(true);
            disp_->highlighted_objects_.insert(ptr);
          }
        }
      }
    }
    catch(...)
    {
      //ROS_WARN("Caught an Ogre exception (probably because there was no CustomParamter!)");
    }
  }


  rviz::InteractionCursorDisplay* disp_;
};


/** This optional class allows you to receive per-result callbacks from
    SceneQuery executions instead of a single set of consolidated results.
@remarks
    You should override this with your own subclass. Note that certain query
    classes may refine this listener interface.
*/
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
      //ROS_INFO("Sphere collides with MoveableObject [%s].",  object->getName().c_str());
      MyVisitor visitor;
      visitor.disp_ = disp_;
      object->visitRenderables( &visitor );
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
};

// -----------------------------------------------------------------------------


InteractionCursorDisplay::InteractionCursorDisplay()
  : Display()
  , nh_("")
  , cursor_shape_(0)
  , dragging_(false)
{
  grabbed_object_.reset();

  update_topic_property_ = new RosTopicProperty( "Update Topic", "",
                                                        ros::message_traits::datatype<interaction_cursor_msgs::InteractionCursorUpdate>(),
                                                        "interaction_cursor_msgs::InteractionCursorUpdate topic to subscribe to.",
                                                        this, SLOT( updateTopic() ));

//  frame_property_ = new TfFrameProperty( "Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
//                                         "The TF frame these axes will use for their origin.",
//                                         this, NULL, true );


  show_cursor_shape_property_ = new BoolProperty("Show Cursor", true,
                                                 "Enables display of cursor shape.",
                                                 this);

  shape_scale_property_ = new FloatProperty( "Cursor Size", 0.2,
                                        "Size of search sphere, in meters.",
                                        this, SLOT( updateShape() ));
  shape_scale_property_->setMin( 0.0001 );

  show_cursor_axes_property_ = new BoolProperty("Show Axes", true,
                                                "Enables display of cursor axes.",
                                                this);

  axes_length_property_ = new FloatProperty( "Axes Length", 0.2,
                                        "Length of each axis, in meters.",
                                        this, SLOT( updateAxes() ));
  axes_length_property_->setMin( 0.0001 );

  axes_radius_property_ = new FloatProperty( "Axes Radius", 0.02,
                                        "Radius of each axis, in meters.",
                                        this, SLOT( updateAxes() ));
  axes_radius_property_->setMin( 0.0001 );
}

InteractionCursorDisplay::~InteractionCursorDisplay()
{
  delete cursor_shape_;
  delete cursor_axes_;
  context_->getSceneManager()->destroySceneNode( cursor_node_ );
}

void InteractionCursorDisplay::updateTopic()
{
  subscriber_update_ = nh_.subscribe<interaction_cursor_msgs::InteractionCursorUpdate>
                              (update_topic_property_->getStdString(), 10,
                              boost::bind(&InteractionCursorDisplay::updateCallback, this, _1));
}

void InteractionCursorDisplay::onInitialize()
{
  //frame_property_->setFrameManager( context_->getFrameManager() );

  cursor_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  cursor_node_->setVisible( isEnabled() );

  cursor_axes_ = new Axes( scene_manager_, cursor_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat() );
  cursor_axes_->getSceneNode()->setVisible( show_cursor_axes_property_->getBool() );

  cursor_shape_ = new Shape( Shape::Sphere, context_->getSceneManager(), cursor_node_);
  cursor_shape_->setScale(Ogre::Vector3(0.2f));
  cursor_shape_->setColor(0.3f, 1.0f, 0.3f, 0.5f);
  cursor_shape_->getRootNode()->setVisible( show_cursor_shape_property_->getBool() );

  // Should this happen onEnable and then get killed later?
  updateTopic();
}

void InteractionCursorDisplay::onEnable()
{
  cursor_node_->setVisible( true );
}

void InteractionCursorDisplay::onDisable()
{
  cursor_node_->setVisible( false );
}

void InteractionCursorDisplay::updateAxes()
{
  cursor_axes_->set( axes_length_property_->getFloat(), axes_radius_property_->getFloat() );
  context_->queueRender();
}


void InteractionCursorDisplay::updateShape()
{
  Ogre::Vector3 shape_scale( 1.05*shape_scale_property_->getFloat());
  cursor_shape_->setScale( shape_scale );
  context_->queueRender();
}

void InteractionCursorDisplay::updateCallback(const interaction_cursor_msgs::InteractionCursorUpdateConstPtr &icu_cptr)
{
  //ROS_INFO("Got a InteractionCursor update!");
  std::string frame = icu_cptr->pose.header.frame_id;
  Ogre::Vector3 position;
  Ogre::Quaternion quaternion;

  if( context_->getFrameManager()->transform(frame, ros::Time(0), icu_cptr->pose.pose, position, quaternion) )
  {
    cursor_node_->setPosition( position );
    cursor_node_->setOrientation( quaternion );

    Ogre::Sphere sphere(position, shape_scale_property_->getFloat());
    clearOldSelections();
    getIntersections(sphere);

    if(icu_cptr->button_state == icu_cptr->GRAB)
    {
      ROS_INFO("Grabbing object!");
      grabObject(position, quaternion);
    }
    else if(icu_cptr->button_state == icu_cptr->KEEP_ALIVE && dragging_)
    {
      ROS_INFO("Updating object pose!");
      updateGrabbedObject(position, quaternion);
    }
    else if(icu_cptr->button_state == icu_cptr->RELEASE && dragging_)
    {
      ROS_INFO("Releasing object!");
      releaseObject();
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
        control->setHoverHighlight(false);
        //disp_->highlighted_objects.insert(ptr);
      }
    }
  }
  highlighted_objects_.clear();
}

void InteractionCursorDisplay::getIntersections(const Ogre::Sphere &sphere)
{
  //ROS_INFO("Requesting intersections at [%.2f %.2f %.2f] with radius %.2f!",
  //         sphere.getCenter().x,
  //         sphere.getCenter().y,
  //         sphere.getCenter().z,
  //         sphere.getRadius());

  Ogre::DefaultSphereSceneQuery ssq(context_->getSceneManager());
  ssq.setSphere(sphere);
  MySceneQueryListener listener;
  listener.disp_ = this;
  ssq.execute(&listener);
}

void InteractionCursorDisplay::grabObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
  if(highlighted_objects_.begin() == highlighted_objects_.end())
    return;

  InteractiveObjectWPtr ptr = *(highlighted_objects_.begin());
  // Remove the object from the set so that we don't un-highlight it later on accident.
  highlighted_objects_.erase(highlighted_objects_.begin());

  if(!ptr.expired())
  {
    boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
    if(control)
    {
      control->setActiveHighlight(true);
      InteractiveMarker* im = control->getParent();
      im->startDragging();
      Ogre::Vector3 r_marker_to_cursor_in_cursor_frame = orientation.Inverse()*(position - im->getPosition());
      position_offset_at_grab_ = r_marker_to_cursor_in_cursor_frame;
      orientation_offset_at_grab_ = orientation.Inverse()*im->getOrientation();
      marker_frame_at_grab_= im->getReferenceFrame();
      std::string marker_name = im->getName();
      ROS_INFO("Grabbed marker [%s] with reference frame [%s], position offset [%.3f %.3f %.3f] and quaternion offset [%.2f %.2f %.2f %.2f]",
               marker_name.c_str(), marker_frame_at_grab_.c_str(),
               position_offset_at_grab_.x, position_offset_at_grab_.y, position_offset_at_grab_.z,
               orientation_offset_at_grab_.x, orientation_offset_at_grab_.y, orientation_offset_at_grab_.z, orientation_offset_at_grab_.w);

      grabbed_object_ = ptr;
      dragging_ = true;

      //disp_->highlighted_objects.insert(ptr);
    }
  }
}

void InteractionCursorDisplay::updateGrabbedObject(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation)
{
  if(!grabbed_object_.expired())
  {
    boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(grabbed_object_.lock());
    if(control)
    {
      const std::string& control_name = control->getName();
      InteractiveMarker* im = control->getParent();
      Ogre::Vector3 r_marker_to_cursor_in_cursor_frame = position_offset_at_grab_;
      Ogre::Quaternion marker_orientation = orientation*orientation_offset_at_grab_;
      Ogre::Vector3 marker_position = orientation*(orientation.Inverse()*position - r_marker_to_cursor_in_cursor_frame);
      im->setPose( marker_position, marker_orientation, control_name );

    }
  }
}

void InteractionCursorDisplay::releaseObject()
{
  if(!grabbed_object_.expired())
  {
    boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(grabbed_object_.lock());
    if(control)
    {
      control->getParent()->stopDragging();
      // Add it back to the set for later un-highlighting.
      highlighted_objects_.insert(grabbed_object_);
    }
  }
  grabbed_object_.reset();
  dragging_ = false;


}

//void InteractionCursorDisplay::

void InteractionCursorDisplay::update( float dt, float ros_dt )
{

}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
// namespace, name, type, base type
PLUGINLIB_DECLARE_CLASS( rviz, InteractionCursorDisplay, rviz::InteractionCursorDisplay, rviz::Display )



