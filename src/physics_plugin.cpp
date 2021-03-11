#include <soft_body_demo/physics_plugin.h>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <Eigen/Geometry>

template <typename T>
inline T read(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}

namespace gazebo
{

void SpringModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  std::cout << "Loading soft body physics" << std::endl;

  ee = model->GetLinks()[0];
  auto base1 = model->GetWorld()->ModelByName("base1")->GetLinks()[0];
  auto base2 = model->GetWorld()->ModelByName("base2")->GetLinks()[0];





  // model params are passed through plugin definition in SDF
  spring_model.initParams(ee->GetWorld()->Physics()->GetMaxStepSize(),  // dt from Gazebo engine
                          read(sdf, "fxy", 10.),
                          read(sdf, "fz", 10.),
                          read(sdf, "mz", 10.),
                          read(sdf, "damping", 0.01),
                          read(sdf, "length", 1.));

  attach = read(sdf, "attach_point", Vector3d());

  // connect update function
  update_event = event::Events::ConnectWorldUpdateBegin(std::bind(&SpringModelPlugin::Update, this));
}

void SpringModelPlugin::Update()
{
  // get pose of end-effector wrt attach point
  auto pose(ee->WorldPose());
  pose.Pos() -= attach;

  // to Eigen pose 3D
  state.updateTranslation(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  state.updateRotation(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

  // get absolute velocity
  auto vel = ee->WorldLinearVel();
  state.twist.linear << vel.X(), vel.Y(), vel.Z();
  vel = ee->WorldAngularVel();
  state.twist.angular << vel.X(), vel.Y(), vel.Z();

  // call model to get wrench
  const auto wrench(spring_model.update(state));

  ee->AddForce({wrench.force.x(), wrench.force.y(), wrench.force.z()});
  ee->AddTorque({wrench.torque.x(), wrench.torque.y(), wrench.torque.z()});
}


}
