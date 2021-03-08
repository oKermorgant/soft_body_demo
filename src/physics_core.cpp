#include <soft_body_demo/physics_core.h>

namespace soft_body_demo
{

using namespace Eigen;

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

void SpringPhysics::initParams(double dt, double fxy, double fz, double mz, double damping, double length)
{
std::cout << "SpringPhysics::initParams" << std::endl;  this->fxy = fxy;
  this->dt = dt;
  this->fz = fz;
  this->mz = mz;
  this->damping = damping;
  this->length = length;
}

Wrench SpringPhysics::update(const State &state)
{
  // we work in body frame
  const auto pose(state.pose.inverse());

  // begin position of spring
  const Vector3d start = pose.translation();
  // begin spring normal
  const Vector3d start_normal = pose.linear().block(0, 2, 3, 1);  // last column (Z) of rotation matrix
  // end position
  Vector3d end; end << 0, 0, -0.05;
  // end normal
  Vector3d end_normal; end_normal << 0, 0, 1;

  // assume we solve some ODE (here fit a polynomial) for physics

  // find 3rd order polynom expressing the spring shape in body frame
  const auto n(4);
  Eigen::MatrixXd M(10, 3*n);
  Eigen::VectorXd v(10);
  M.setZero();
  v.setZero();

  // x0
  M(0,0) = 1;
  v(0) = start.x();
  // y0
  M(1,n) = 1;
  v(1) = start.y();
  // z0
  M(2,2*n) = 1;
  v(2) = start.z();

  // (xf, yf, zf) = end
  for(int i = 0; i < n; ++i)
    M(3,i) = M(4, n+i) = M(5, 2*n+i) = 1;
  v(3) = end.x();
  v(4) = end.y();
  v(5) = end.z();

  // X/Z0 angle
  M(6,1) = start_normal.z();
  M(6, 2*n+1) = -start_normal.x();
  // Y/Z0 angle
  M(7,n+1) = start_normal.z();
  M(7, 2*n+1) = -start_normal.y();

  // X/Zf angle
  for(int i = 1; i < n; ++i)
  {
    M(8,i) = i*end_normal.z();
    M(8, 2*n+i) = -i*end_normal.x();
  }
  // Y/Zf angle
  for(int i = 1; i < n; ++i)
  {
    M(9,n+i) = i*end_normal.z();
    M(9, 2*n+i) = -i*end_normal.y();
  }

  // solve underdetermined and get polynomials for X Y Z
  Eigen::VectorXd xyz(M.transpose() * (M*M.transpose()).householderQr().solve(v));

  // write coefficients in message and send to visual plugin
  for(size_t i = 0; i < n; ++i)
  {
    shape.xc[i] = xyz[i];
    shape.yc[i] = xyz[n+i];
    shape.zc[i] = xyz[2*n+i];
  }

  shape_publisher.publish(shape);

  // basic decoupled damped spring
  Wrench wrench(-damping * state.twist.linear, -damping * state.twist.angular);

  wrench.force.x() -= fxy * state.pose.translation().x();
  wrench.force.y() -= fxy * state.pose.translation().y();
  wrench.force.z() -= fz * (state.pose.translation().z() - length);

  wrench.torque.z() += mz*pose.linear().eulerAngles(0,1,2).z();
  return wrench;

}

}
