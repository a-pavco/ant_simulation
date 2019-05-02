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
  class AntControl : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      this->jointFL = this->model->GetJoint("ant::FLLJ1");
      this->jointML = this->model->GetJoint("ant::MLLJ1");
      this->jointHL = this->model->GetJoint("ant::HLLJ1");
      this->jointFR = this->model->GetJoint("ant::FRLJ1");
      this->jointMR = this->model->GetJoint("ant::MRLJ1");
      this->jointHR = this->model->GetJoint("ant::HRLJ1");

      this->pidFL = common::PID(20.0, 0.1, 1);
      this->pidML = common::PID(20.0, 0.1, 1);
      this->pidHL = common::PID(20.0, 0.1, 1);
      this->pidFR = common::PID(20.0, 0.1, 1);
      this->pidMR = common::PID(20.0, 0.1, 1);
      this->pidHR = common::PID(20.0, 0.1, 1);

      this->model->GetJointController()->SetPositionPID(this->jointFL->GetScopedName(), this->pidFL);
      this->model->GetJointController()->SetPositionPID(this->jointML->GetScopedName(), this->pidML);
      this->model->GetJointController()->SetPositionPID(this->jointHL->GetScopedName(), this->pidHL);
      this->model->GetJointController()->SetPositionPID(this->jointFR->GetScopedName(), this->pidFR);
      this->model->GetJointController()->SetPositionPID(this->jointMR->GetScopedName(), this->pidMR);
      this->model->GetJointController()->SetPositionPID(this->jointHR->GetScopedName(), this->pidHR);

      this->leftTripod = true;

      this->SetJointPositions(100.0);
         

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo"));
      

      // Create a topic name
      std::string ant_topicName = "/ant_topic";
      // ant node
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            ant_topicName,
            1,
            boost::bind(&AntControl::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&AntControl::QueueThread, this));

      ROS_WARN("Loaded AntControl Plugin with parent...%s", this->model->GetName().c_str());  
      ROS_WARN("Joint count...%s", std::to_string(this->model->GetJointCount()).c_str()); 
      
    }

    
    public: void SetJointPositions(const double &_position) 
    {
      if (leftTripod) {
        ROS_WARN("Setting left tripod joints poses.");
        this->model->GetJointController()->SetPositionTarget(this->jointFL->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->jointMR->GetScopedName(), -_position);
        this->model->GetJointController()->SetPositionTarget(this->jointHL->GetScopedName(), _position);
        leftTripod = false;
      }
      else {
        ROS_WARN("Setting right tripod joints poses.");
        this->model->GetJointController()->SetPositionTarget(this->jointML->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->jointFR->GetScopedName(), -_position);
        this->model->GetJointController()->SetPositionTarget(this->jointHR->GetScopedName(), -_position);
        leftTripod = true;
      }
    	
    }
    
     
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
    	ROS_WARN("Got RosMsg.");
      this->SetJointPositions(_msg->data);
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
    
   

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Pointer to the joints.
	 private: physics::JointPtr jointFL;
	private: physics::JointPtr jointML;
	private: physics::JointPtr jointHL;
	private: physics::JointPtr jointFR;
	private: physics::JointPtr jointMR;
	private: physics::JointPtr jointHR;

	private: common::PID pidFL;
	private: common::PID pidML;
	private: common::PID pidHL;
	private: common::PID pidFR;
	private: common::PID pidMR;
	private: common::PID pidHR;

  bool leftTripod;
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AntControl)
}
