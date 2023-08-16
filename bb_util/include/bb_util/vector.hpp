/**
   \file vector.hpp
   \brief Provides an implementation of the bb_util::Vec2D class
*/

#pragma once

#include <cmath>
#include "bb_util/vec2d_msg.h"


namespace bb_util{
  /**
     \brief Provides a simple 2D vector type for convenience.
     
     Early in development there was lots of work with 2D vectors which
     required both cartesian and polar representations to make different
     calculations easier. This class was constructed to make that process
     easier.

     \note This class is note instantiated in the standard way. 
     One should use `bb_util::Vec2D::init_cartesian(x,y)` or
     `bb_util::Vec2D::init_polar(r,theta)` which will return the
     object. It is assumed that theta is given in radians.
  */
  class Vec2D{
  private:
    double x, y, theta, r;

    /**
       Private constructor formulation.
       Static methods below provide the initialisation functions
       for Vec2D objects. This allows initialisation using either
       cartesian coordinates or polar coordinates.
     */
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
    static bb_util::vec2d_msg to_message(const Vec2D&);
    static Vec2D to_vector(const bb_util::vec2d_msg&);

    // Operator overloads
    Vec2D operator+(const Vec2D& v);
    Vec2D operator+=(const Vec2D& v);
    Vec2D operator/(const double d);
    Vec2D operator*(const double d);
  };

  //
  // Implementation
  //

  /**
     Generate a Vec2D object using cartesian coordinates.
     \param x The X coordinate
     \param y The Y coordinate
     \return The 2D vector [x,y]
  */
  inline Vec2D Vec2D::init_cartesian(double x, double y){
    return Vec2D(sqrt(x*x + y*y), atan2(y,x));
  }

    /**
     Generate a Vec2D object using polar coordinates.
     \param r The vector magnitude
     \param theta The angle from the x axis.
     \return The 2D vector [r*cos(theta),r*sin(theta)]
  */
  inline Vec2D Vec2D::init_polar(double r, double theta){
    return Vec2D(r, theta);
  }

  /**
     Updates internal cartesian representation to keep it consistent
     with the polar one.
   */
  void Vec2D::update_cartesian(){
      x = r * cos(theta);
      y = r * sin(theta);
  }

  /**
     Updates internal polar representation to keep it consistent
     with the cartesian one.
  */
  void Vec2D::update_polar(){
    r = sqrt(x*x + y*y);
    theta = atan2(y,x);
  }

  /**
     Update the vector using cartesian coordinates.
     \param x Cartesian x.
     \param y Cartesian y
  */
  void Vec2D::set_cartesian(double x, double y){
    this->x = x;
    this->y = y;
    this->update_polar();
  }


  /**
     Update the vector using polar coordinates.
     \param r Magnitude
     \param theta Angle from x-axis
  */
  void Vec2D::set_polar(double r, double theta){
    this->r = r;
    this->theta = theta;
    this->update_cartesian();
  }

  /** Set just the x component */
  void Vec2D::set_x(double x){ this->set_cartesian(x, this->y); }
  
  /** Set just the y component */
  void Vec2D::set_y(double y){ this->set_cartesian(this->x, y); }

  /** Set just the magnitude component */
  void Vec2D::set_r(double r){ this->set_polar(r, this->theta); }

  /** Set just the angular component */
  void Vec2D::set_theta(double theta){ this->set_polar(this->r, theta); }

  /** Get the angular component in radians */
  double Vec2D::get_theta() const { return theta; }

  /** Get the magnitude */
  double Vec2D::get_r() const { return r; }

  /** Get the x coordinate */
  double Vec2D::get_x() const { return x; }

  /** Get the y coordinate */
  double Vec2D::get_y() const { return y; }

  /**
     Convert to bb_util::vec2d_msg ROS message format
     \param vec A reference to the vector to be transformed
     \return The equivalent ROS message.
  */
  inline bb_util::vec2d_msg Vec2D::to_message(const Vec2D& vec){
    bb_util::vec2d_msg msg;
    msg.r = vec.get_r();
    msg.theta = vec.get_theta();
    msg.x = vec.get_x();
    msg.y = vec.get_y();
    return msg;
  }

  /**
     Convert a bb_util::vec2d_msg to a Vec2D object.
     \param msg The ROS message.
     \return The Vec2D object.
   */
  inline Vec2D Vec2D::to_vector(const bb_util::vec2d_msg& msg){
    return Vec2D::init_polar(msg.r, msg.theta);
  }


  /** Overload `+` for vector addition*/
  Vec2D Vec2D::operator+(const Vec2D& v){
    double x = this->get_x();
    double y = this->get_y();

    x += v.get_x();
    y += v.get_y();

    return Vec2D::init_cartesian(x, y);
  }

  /** Overload `+=`*/
  Vec2D Vec2D::operator+=(const Vec2D& v){
    *this = *this + v;
    return *this;
  }

  /** Overload `/` for scalar division*/
  Vec2D Vec2D::operator/(const double d){
    double x = this->get_x() / d;
    double y = this->get_y() / d;

    return Vec2D::init_cartesian(x, y);
  }

  /** Overload `*` for scalar multiplication */
  Vec2D Vec2D::operator*(const double d){
    double x = this->get_x() * d;
    double y = this->get_y() * d;

    return Vec2D::init_cartesian(x, y);
  }
  
} // namespace bb_util
