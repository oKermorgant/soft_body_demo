#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_srvs/Empty.h>

#include <soft_body_demo/physics_core.h>

// helper structure to communicate with Gazebo through services
// change messages types (pose / twist / wrench) to Eigen ones
class GazeboIO
{
public:
  GazeboIO(ros::NodeHandle &nh, std::string model_name, std::string link_name, double dt)
  {
    state_srv = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    wrench_srv = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    std_srvs::Empty empty;
    nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world").call(empty);
    ros::spinOnce();

    // prepare messages
    state_req.model_name = model_name;
    state_req.relative_entity_name = "world";

    wrench_req.body_name = model_name + "::" + link_name;
    wrench_req.reference_frame = model_name;
    wrench_req.duration.fromSec(dt);
  }

  soft_body_demo::State getState()
  {
    state_srv.call(state_req, state_res);
    // parse output
    state.updateTranslation(state_res.pose.position.x, state_res.pose.position.y, state_res.pose.position.z);
    state.updateRotation(state_res.pose.orientation.w, state_res.pose.orientation.x, state_res.pose.orientation.y, state_res.pose.orientation.z);
    msg2Eigen(state_res.twist.linear, state.twist.linear);
    msg2Eigen(state_res.twist.angular, state.twist.angular);

    return state;
  }

  void applyWrench(const soft_body_demo::Wrench &wrench)
  {
    // to msg
    Eigen2Msg(wrench.force, wrench_req.wrench.force);
    Eigen2Msg(wrench.torque, wrench_req.wrench.torque);

    Eigen2Msg(state.pose.translation(), wrench_req.reference_point);

    wrench_req.start_time = ros::Time::now();

    wrench_srv.call(wrench_req, wrench_res);
  }

private:
  soft_body_demo::State state;
  ros::ServiceClient state_srv, wrench_srv;
  gazebo_msgs::GetModelStateRequest state_req;
  gazebo_msgs::GetModelStateResponse state_res;

  gazebo_msgs::ApplyBodyWrenchRequest wrench_req;
  gazebo_msgs::ApplyBodyWrenchResponse wrench_res;

  inline static void msg2Eigen(const geometry_msgs::Vector3 &msg, Eigen::Vector3d &vec)
  {
    vec << msg.x, msg.y, msg.z;
  }
  template <class MsgVector3>
  inline static void Eigen2Msg(const Eigen::Vector3d &vec, MsgVector3 &msg)
  {
    msg.x = vec.x();
    msg.y = vec.y();
    msg.z = vec.z();
  }
};



int main (int argc, char** argv)
{
  ros::init(argc, argv, "mass_spring_physics");
  ros::NodeHandle nh;

  ros::NodeHandle priv("~");
  // sim rate
  const double dt = 0.01;priv.param("dt", 0.001);
  ros::Rate rate(1./dt);

  // init model from passed params
  soft_body_demo::SpringPhysics model;
  model.initParams(dt,
                   priv.param("fxy", 10.),
                   priv.param("fz", 50.),
                   priv.param("mz", 0.1),
                   priv.param("damping", .1),
                   priv.param("length", 1.));


  // setup services
  GazeboIO io(nh, "mass_spring", "ee", dt);

  while(ros::ok())
  {
    // get -> compute -> apply wrench
    io.applyWrench(model.update(io.getState()));

    ros::spinOnce();
    rate.sleep();
  }







}
