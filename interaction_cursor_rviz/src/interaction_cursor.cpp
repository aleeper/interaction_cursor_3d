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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

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
    : sm_(0) {}

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

    try
    {
      Ogre::Vector4 vec = rend->getCustomParameter(PICK_COLOR_PARAMETER);
      if(sm_)
      {
        Ogre::ColourValue colour(vec.x, vec.y, vec.z, 1.0);
        CollObjectHandle handle = sm_->colourToHandle(colour);

        rviz::SelectionHandler* handler = sm_->getHandler(handle);
        if(handle)
        {
          InteractiveObjectWPtr ptr = handler->getInteractiveObject();

          // TODO so once we set a highlight, how do we UNset it? ...
          //weak_ptr<Fruit> fruit = weak_ptr<Fruit>(dynamic_pointer_cast<Fruit>(food.lock());
          boost::shared_ptr<InteractiveMarkerControl> control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
          if(control)
          {
            control->setHoverHighlight(true);
          }
        }
      }
    }
    catch(...)
    {
      ROS_WARN("Caught an Ogre exception (probably because there was no CustomParamter!)");
    }
  }


  rviz::SelectionManager* sm_;
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
    virtual ~MySceneQueryListener() { }
    /** Called when a MovableObject is returned by a query.
    @remarks
        The implementor should return 'true' to continue returning objects,
        or 'false' to abandon any further results from this query.
    */
    virtual bool queryResult(Ogre::MovableObject* object)
    {
      ROS_INFO("Sphere collides with MoveableObject [%s].",  object->getName().c_str());
      MyVisitor visitor;
      visitor.sm_ = context_->getSelectionManager();
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
      ROS_INFO("Sphere collides with WorldFragment type [%d].", fragment->fragmentType );
      return true;
    }
    rviz::DisplayContext* context_;
};

// -----------------------------------------------------------------------------


InteractionCursorDisplay::InteractionCursorDisplay()
  : Display()
  , axes_( 0 )
{
  update_topic_property_ = new RosTopicProperty( "Update Topic", "",
                                                        ros::message_traits::datatype<interaction_cursor_msgs::InteractionCursorUpdate>(),
                                                        "interaction_cursor_msgs::InteractionCursorUpdate topic to subscribe to.",
                                                        this, SLOT( updateTopic() ));

  frame_property_ = new TfFrameProperty( "Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
                                         "The TF frame these axes will use for their origin.",
                                         this, NULL, true );

//  length_property_ = new FloatProperty( "Length", 1.0,
//                                        "Length of each axis, in meters.",
//                                        this, SLOT( updateShape() ));
//  length_property_->setMin( 0.0001 );

  radius_property_ = new FloatProperty( "Radius", 0.1,
                                        "Radius of search sphere, in meters.",
                                        this, SLOT( updateShape() ));
  radius_property_->setMin( 0.0001 );
}

InteractionCursorDisplay::~InteractionCursorDisplay()
{
  if(axes_)
    delete axes_;
}

void InteractionCursorDisplay::onInitialize()
{
  frame_property_->setFrameManager( context_->getFrameManager() );

  //axes_ = new Axes( scene_manager_, 0, length_property_->getFloat(), radius_property_->getFloat() );
  //axes_->getSceneNode()->setVisible( isEnabled() );

  axes_ = new Shape( Shape::Sphere, context_->getSceneManager(), 0);
  axes_->setScale(Ogre::Vector3(1.0f, 1.0f, 1.0f));
  axes_->setColor(0.3f, 1.0f, 0.3f, 0.5f);
  axes_->getRootNode()->setVisible( isEnabled() );
}

void InteractionCursorDisplay::onEnable()
{
  axes_->getRootNode()->setVisible( true );
}

void InteractionCursorDisplay::onDisable()
{
  axes_->getRootNode()->setVisible( false );
}

void InteractionCursorDisplay::updateShape()
{
  //axes_->set( length_property_->getFloat(), radius_property_->getFloat() );
  float s = 1.05*radius_property_->getFloat();
  axes_->setScale(Ogre::Vector3(s,s,s));
  context_->queueRender();
}

void InteractionCursorDisplay::updateCallback(const interaction_cursor_msgs::InteractionCursorUpdateConstPtr &icu_cptr)
{
  ROS_INFO("Got a InteractionCursor update!");
  Ogre::Vector3 last_position;
  Ogre::Quaternion last_quaternion;

  context_->getFrameManager()->transform(icu_cptr->pose.header.frame_id, ros::Time(0), icu_cptr->pose.pose, last_position, last_quaternion);

  // TODO magic number!
  Ogre::Sphere sphere(last_position, radius_property_->getFloat());
  getIntersections(sphere);

}

void InteractionCursorDisplay::getIntersections(const Ogre::Sphere &sphere)
{
  ROS_INFO("Requesting intersections at [%.2f %.2f %.2f] with radius %.2f!",
           sphere.getCenter().x,
           sphere.getCenter().y,
           sphere.getCenter().z,
           sphere.getRadius());

  Ogre::DefaultSphereSceneQuery ssq(context_->getSceneManager());
  ssq.setSphere(sphere);
  MySceneQueryListener listener;
  listener.context_ = context_;
  ssq.execute(&listener);
}

void InteractionCursorDisplay::update( float dt, float ros_dt )
{
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  position.x = 0;
  position.y = -3*(1 + sin(ros::Time::now().toSec()));
  position.z = 0;

  axes_->setPosition( position );
  axes_->setOrientation( orientation );

  Ogre::Sphere sphere(position, radius_property_->getFloat());
  getIntersections(sphere);


//  if( context_->getFrameManager()->getTransform( frame, ros::Time(), position, orientation ))
//  {
//    axes_->setPosition( position );
//    axes_->setOrientation( orientation );
//    setStatus( StatusProperty::Ok, "Transform", "Transform OK" );
//  }
//  else
//  {
//    std::string error;
//    if( context_->getFrameManager()->transformHasProblems( frame, ros::Time(), error ))
//    {
//      setStatus( StatusProperty::Error, "Transform", QString::fromStdString( error ));
//    }
//    else
//    {
//      setStatus( StatusProperty::Error,
//                 "Transform",
//                 "Could not transform from [" + qframe + "] to Fixed Frame [" + fixed_frame_ + "] for an unknown reason" );
//    }
//  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
// namespace, name, type, base type
PLUGINLIB_DECLARE_CLASS( rviz, InteractionCursorDisplay, rviz::InteractionCursorDisplay, rviz::Display )



