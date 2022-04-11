#include "ros/ros.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "geometry_msgs/Wrench.h"
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <tgmath.h> 
using namespace std;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

  	private: ros::NodeHandle* _node_handle;
    private: ros::Subscriber  _ft_sub;
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::LinkPtr  _base_link;
    private: geometry_msgs::Wrench _wrench;
    private: string _link_name;
    private: int _body_frame_ref;
    private: int _debug;
    private: string _ft_topic_name;
    private: ignition::transport::Node node_ign;
    private: ignition::msgs::Marker markerMsg;
    private: ignition::msgs::Marker markerMsg2;

    public: void FTs_cb( geometry_msgs::Wrench w ) {
      _wrench = w;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      
      _node_handle = new ros::NodeHandle();	
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      _wrench.force.x = 0.0;
      _wrench.force.y = 0.0;
      _wrench.force.z = 0.0;

      _wrench.torque.x = 0.0;
      _wrench.torque.y = 0.0;
      _wrench.torque.z = 0.0;
      
      _debug = 0;
      _body_frame_ref = 0;
      _debug = _sdf->Get<int>("debug"); 
      _body_frame_ref = _sdf->Get<int>("body_frame"); 
      _link_name = _sdf->Get<string>("link_name");
      _ft_topic_name = _sdf->Get<string>("topic_name");
      _ft_sub = _node_handle->subscribe(_ft_topic_name, 1, &ModelPush::FTs_cb, this );

    
      // Visual Marker
       markerMsg.set_ns("default");
       markerMsg.set_id(0);
       markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
       markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
       ignition::msgs::Set(markerMsg.mutable_scale(),
                       ignition::math::Vector3d(0.065, 0.065, 0.065));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(0, 0, 0.0));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, -1, 0.0));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 1, 0.0));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(0, 0, 0.01));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, -1, 0.01));
       ignition::msgs::Set(markerMsg.add_point(),
                       ignition::math::Vector3d(1, 1, 0.01));

 
   
   
   
       markerMsg2.set_ns("default");
       markerMsg2.set_id(1);
       markerMsg2.set_action(ignition::msgs::Marker::ADD_MODIFY);
       markerMsg2.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
       ignition::msgs::Set(markerMsg2.mutable_scale(),
                       ignition::math::Vector3d(0.16, 0.05, 0.2));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(0, -0.5, 0.0));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(1, -0.5, 0.0));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(0, 0.5, 0.0));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(1, 0.5, 0.0));

       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(0, -0.5, 0.01));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(1, -0.5, 0.01));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(0, 0.5, 0.01));
       ignition::msgs::Set(markerMsg2.add_point(),
                       ignition::math::Vector3d(1, 0.5, 0.01));

   
       ignition::msgs::Material *matMsg = markerMsg.mutable_material();
       ignition::msgs::Material *matMsg2 = markerMsg2.mutable_material();
       matMsg->mutable_script()->set_name("Gazebo/YellowGlow");
       matMsg2->mutable_script()->set_name("Gazebo/YellowGlow");
      
    }

    // Called by the world update start event
    public: void OnUpdate() {   
      common::Time currTime = this->model->GetWorld()->SimTime();
      // Apply a small linear velocity to the model.
      //if (currTime>=3) {
        _base_link = model->GetLink( _link_name );
        if ( _base_link == NULL ) {
          cout << "base link null!" << endl;
        }
        else { 
          if ( _body_frame_ref == 1 ) {
            _base_link->AddRelativeForce(ignition::math::Vector3d( _wrench.force.x, _wrench.force.y, _wrench.force.z ));
            _base_link->AddRelativeTorque(ignition::math::Vector3d( _wrench.torque.x, _wrench.torque.y, _wrench.torque.z ));
          }
          else {
            _base_link->AddForce(ignition::math::Vector3d( _wrench.force.x, _wrench.force.y, _wrench.force.z ));
            _base_link->AddTorque(ignition::math::Vector3d( _wrench.torque.x, _wrench.torque.y, _wrench.torque.z ));
          }
        }	
      
      // Adding visual marker
        if( _wrench.force.x>0 || _wrench.force.y>0 || _wrench.force.y>0 )
        {
           ignition::math::Vector3d pos= _base_link->WorldPose().Pos();	
      
           double alfa=atan2(_wrench.force.y, _wrench.force.x);
           double theta=atan2(_wrench.force.z,_wrench.force.x);
   
           ignition::msgs::Set(markerMsg.mutable_pose(),
                               ignition::math::Pose3d(pos.X(),pos.Y(),pos.Z(),1.57,theta,3.14+alfa));
   
           ignition::msgs::Set(markerMsg2.mutable_pose(),
                               ignition::math::Pose3d(pos.X(),pos.Y(),pos.Z(),1.57,theta,3.14+alfa));
   
           node_ign.Request("/marker", markerMsg);
           node_ign.Request("/marker", markerMsg2);
        }
      


      //if (_debug ) {
      //  ignition::math::Vector3d applied_force =_base_link->RelativeForce(); //(ignition::math::Vector3d( _wrench.force.x, _wrench.force.y, _wrench.force.z ));
      //  ignition::math::Vector3d applied_torque =_base_link->RelativeTorque(); //(ignition::math::Vector3d( _wrench.force.x, _wrench.force.y, _wrench.force.z ));
      //  cout << "Desired force: " <<  _wrench.force.x << " " << _wrench.force.y << " " << _wrench.force.z << endl;
      //  cout << "Applied force: " << applied_force[0] << " " << applied_force[1] << " " << applied_force[2]  << endl;
      //  cout << "Desired torque: " <<  _wrench.torque.x << " " << _wrench.torque.y << " " << _wrench.torque.z << endl;
      //  cout << "Applied torque: " << applied_torque[0] << " " << applied_torque[1] << " " << applied_torque[2]  << endl;
      //}


    }

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
