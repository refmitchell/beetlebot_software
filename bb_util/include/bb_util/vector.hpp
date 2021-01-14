#pragma once

#include <cmath>
#include "bb_util/polar_vector.h"

/**
   @file vector.hpp
   @brief Provides a convenient vector container (polar/cartesian).
*/

namespace bb_util{
  /**
     Provides a simple 2D vector type for convenience.
  */
  class Vec2D{
  private:
    double x, y, theta, r;

    Vec2D(double m, double angle) : r(m), theta(angle){
      x = r * cos(theta);
      y = r * sin(theta);
    }

    void update_cartesian();
    void update_polar();

  public:
    static Vec2D init_cartesian(double, double);
    static Vec2D init_polar(double, double);

    void set_cartesian(double, double);
    void set_polar(double, double);

    void set_r(double);
    void set_theta(double);

    void set_x(double);
    void set_y(double);

    double get_x() const;
    double get_y() const;

    double get_r() const;
    double get_theta()const;

    // ROS message conversion
    static bb_util::polar_vector to_message(const Vec2D&);
    static Vec2D to_vector(const bb_util::polar_vector&);
  };

  //
  // Implementation
  //
  inline Vec2D Vec2D::init_cartesian(double x, double y){
    return Vec2D(sqrt(x*x + y*y), atan(y/x));
  }

  inline Vec2D Vec2D::init_polar(double r, double theta){
    return Vec2D(r, theta);
  }

  void Vec2D::update_cartesian(){
      x = r * cos(theta);
      y = r * sin(theta);
  }

  void Vec2D::update_polar(){
    r = sqrt(x*x + y*y);
    theta = atan(y/x);
  }

  //
  // Setters, as this is a convenience class
  // no error checking. All angular inputs are expected
  // in radians.
  //

  /**
     Update the vector using cartesian coordinates.
     @param x Cartesian x.
     @param y Cartesian y
  */
  void Vec2D::set_cartesian(double x, double y){
    this->x = x;
    this->y = y;
    this->update_polar();
  }


  /**
     Update the vector using polar coordinates.
     @param r Magnitude
     @param theta Angle from x-axis
  */
  void Vec2D::set_polar(double r, double theta){
    this->r = r;
    this->theta = theta;
    this->update_cartesian();
  }

  //
  // Component setters
  //
  void Vec2D::set_x(double x){ this->set_cartesian(x, this->y); }
  void Vec2D::set_y(double y){ this->set_cartesian(this->x, y); }

  void Vec2D::set_r(double r){ this->set_polar(r, this->theta); }
  void Vec2D::set_theta(double theta){ this->set_polar(this->r, theta); }

  //
  // Component getters
  //
  double Vec2D::get_theta() const { return theta; }
  double Vec2D::get_r() const { return r; }

  double Vec2D::get_x() const { return x; }
  double Vec2D::get_y() const { return y; }

  //
  // ROS Message/Class conversions
  //
  inline bb_util::polar_vector Vec2D::to_message(const Vec2D& vec){
    bb_util::polar_vector msg;
    msg.r = vec.get_r();
    msg.theta = vec.get_theta();
    return msg;
  }

  inline Vec2D Vec2D::to_vector(const bb_util::polar_vector& msg){
    return Vec2D::init_polar(msg.r, msg.theta);
  }

} // namespace bb_util
