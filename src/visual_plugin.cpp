#include <soft_body_demo/visual_plugin.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/DynamicLines.hh>

template <typename T>
inline T read(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}

// evaluate a polynomial
inline double eval(const soft_body_demo::Coefs &coefs, double x)
{
  double out(0);
  double exp(1);
  for(auto &coef: coefs)
  {
    out += coef * exp;
    exp *= x;
  }
  return out;
}

Vector3d getPoint(const soft_body_demo::ShapeMsg &msg, double t)
{
  return {eval(msg.xc, t), eval(msg.yc, t), eval(msg.zc, t)};
}

namespace gazebo
{

void SpringVisualPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{
  std::cout << "Loading soft body visual" << std::endl;
  radius = read(sdf, "radius", 0.1);
  inv_scale.X(1./parent->Scale().X());
  inv_scale.Y(1./parent->Scale().Y());
  inv_scale.Z(1./parent->Scale().Z());

  visual = parent;
  lines = visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  lines->setMaterial("Gazebo/Blue");
  lines->setVisibilityFlags(GZ_VISIBILITY_GUI);
  for(int i = 0; i <= points; ++i)
    lines->AddPoint({}, ignition::math::Color::Blue);

  // connect update function
  update_event = event::Events::ConnectPreRender(std::bind(&SpringVisualPlugin::Update, this));
}

void SpringVisualPlugin::Update()
{
  auto msg = shape_listener.lastVisual();

  double t(0);
  const static double step(1./points);
  for(int i = 0; i <= points; ++i)
  {
    lines->SetPoint(i, inv_scale*getPoint(msg, t)); // element-wise scale multiply
    t += step;
  }
  lines->Update();
}

}
