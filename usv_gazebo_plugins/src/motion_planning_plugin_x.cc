#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelMotion_x : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelMotion_x::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();
      
      // Create a topic name
      std::string frequancy_topicName_x = "/frequancy_x";
      std::string magnitude_topicName_x = "/magnitude_x";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motion_rosnode",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("motion_rosnode"));
      
      // Freq
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            frequancy_topicName_x,
            1,
            boost::bind(&ModelMotion_x::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelMotion_x::QueueThread, this));
        
        
      // Magnitude
      ros::SubscribeOptions so2 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            magnitude_topicName_x,
            1,
            boost::bind(&ModelMotion_x::OnRosMsg_Magn, this, _1),
            ros::VoidPtr(), &this->rosQueue2);
      this->rosSub2 = this->rosNode->subscribe(so2);
      
      // Spin up the queue helper thread.
      this->rosQueueThread2 =
        std::thread(std::bind(&ModelMotion_x::QueueThread2, this));
         
      ROS_WARN("Loaded Motion Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", this->model->GetName().c_str());
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;
      
      double max_delta = 0.0;
      
      if (this->x_axis_freq != 0.0)
      {
        max_delta = 1.0 / this->x_axis_freq;
      }
      
      double magnitude_speed = this->x_axis_magn;
      
      
      if (delta > max_delta && delta != 0.0)
      {
        // We change Direction
        this->direction = this->direction * -1;
        this->old_secs = new_secs;
        ROS_DEBUG("Change Direction...");
      }
      
      double speed = magnitude_speed * this->direction;
      
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(speed, 0, 0));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    }
    
    
    public: void SetFrequency(const double &_freq)
    {
      this->x_axis_freq = _freq;
      ROS_WARN("x_axis_freq >> %f", this->x_axis_freq);
    }
    
    public: void SetMagnitude(const double &_magn)
    {
      this->x_axis_magn = _magn;
      ROS_WARN("x_axis_magn >> %f", this->x_axis_magn);
    }
    
    
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetFrequency(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    public: void OnRosMsg_Magn(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetMagnitude(_msg->data);
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread2()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue2.callAvailable(ros::WallDuration(timeout));
      }
    }
    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Direction Value
    int direction = 1;
    // Frequency 
    double x_axis_freq = 0.1;
    // Magnitude of the Oscilations
    double x_axis_magn = 30.0;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelMotion_x)
}
