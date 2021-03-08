#ifndef SOFT_BODY_PHYSICS_PLUGIN_H
#define SOFT_BODY_PHYSICS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <soft_body_demo/physics_core.h>

using ignition::math::Vector3d;

namespace gazebo
{

class SpringModelPlugin : public ModelPlugin
{
public:
    SpringModelPlugin() {}
    ~SpringModelPlugin()
    {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void Update();

protected:
    soft_body_demo::SpringPhysics spring_model;
    soft_body_demo::State state;
    physics::LinkPtr ee;
    double length;
    double mxy, mz, fxy, fz, damping;
    Vector3d attach;
    event::ConnectionPtr update_event;
};

GZ_REGISTER_MODEL_PLUGIN(SpringModelPlugin)
}


#endif // SOFT_BODY_PHYSICS_PLUGIN_H
