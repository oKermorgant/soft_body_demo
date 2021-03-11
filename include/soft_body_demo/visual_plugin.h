#ifndef SOFT_BODY_VISUAL_PLUGIN_H
#define SOFT_BODY_VISUAL_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <soft_body_demo/shape_listener.h>

using ignition::math::Vector3d;

namespace gazebo
{

class SpringVisualPlugin : public VisualPlugin
{
public:
    SpringVisualPlugin() {}
    ~SpringVisualPlugin()
    {}

    void Load(rendering::VisualPtr parent, sdf::ElementPtr _sdf);
    void Update();

protected:
    rendering::VisualPtr visual;
    soft_body_demo::ShapeListener shape_listener;
    double radius;
    event::ConnectionPtr update_event;
    rendering::DynamicLines* lines;
    int points = 500;
    Vector3d inv_scale;
};
GZ_REGISTER_VISUAL_PLUGIN(SpringVisualPlugin)
}


#endif // SOFT_BODY_VISUAL_PLUGIN_H
