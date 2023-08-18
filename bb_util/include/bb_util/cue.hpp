/**
   \file cue.hpp
   \brief Provides an implementation of the bb_util::Cue class.

   Cues should have the same representation across the codebase so
   the definition is given here.
*/

#ifndef BBUTIL_CUE
#define BBUTIL_CUE

#include "bb_util.h"
#include "bb_util/cue_msg.h"

#include <sstream>

namespace bb_util{
  /**
     \brief Global Cue class to keep representation consistent.
     \note This class was developed long before the conceptual progress which 
     lead to the neural model in Mitchell et al. (2023). As a result, some properties
     such as sensitivity or relative_weight are included here which are no longer
     so relelevant to the cue integration problem in the context of the beetlebot.
  */
  class Cue{
  private:
    std::string type =  "";
    double sensitivity = 1;
    double contrast = 0;
    double relative_weight = 0;
    double theta = 0;

  public:
    /**
       \brief Constructor. 

       Essentially this class represents a cue as a polar vector with
       some additional conceptual wrapping. Generally, the important
       parameters are contrast (magnitude, expected to be between 0
       and 1) and azimuth (angle, assumed to be in radians).

       \param type The type of cue, this is primarily used to determine equality.
       \param sensitivity The agent's sensitivity to this cue.
       \param contrast The contrast of the cue (following Mitchell et al. (2023))
       \param azimuth The angle of the cue.
    */
    Cue(std::string type,
        double sensitivity,
        double contrast,
        double azimuth):
      type(type),
      sensitivity(sensitivity),
      contrast(contrast),
      theta(azimuth) {}

    /** Get the contrast parameter */
    double getContrast(){ return this->contrast; }
    /** Get the type parameter */
    std::string getType(){ return this->type; }
    /** Get the relative weight (must be set in advance) */
    double getRelativeWeight(){ return this->relative_weight; }
    /** Get the azimuth of the cue*/
    double getTheta(){ return this->theta; }
    /** Get the sensitivity parameter */
    double getSensitivity(){ return this->sensitivity; }

    /** 
        Set the relative weight. 
        
        \note This function is exclusively used by the bb_computation
        cue_manager node.
    */
    void setRelativeWeight(double rw){ relative_weight = rw; }

    /**
       Set the contrast parameter
       \param r The desired contrast (between 0 and 1 inclusive).
       \warning If r > 1 or r < 0 then the function quitely ignores
       r and the contrast will not be updated.
     */
    void setContrast(double r){
      // If the attempted set is invalid, don't update the value.
      this->contrast = (r >= 0 && r <= 1) ? r : this->contrast;
    }

    /**
       Set the cue azimuth.
       \param t The angle of the cue.

       \warning It is assumed that the value is in radians. Note also
       that the passed angle is \b not mapped into 0,2pi or -pi,pi.
       It is up to the caller to be consistent with angular representations.
    */
    void setAzimuth(double t){
      // Assume valid value in radians.
      this->theta = t;
    }


    /**
       Translate a bb_util::cue_msg to a bb_util::Cue object.
       \param cue_msg The ROS message
       \return An equivalent Cue object
     */
    static Cue toCue(bb_util::cue_msg cue_msg){
      Cue result(cue_msg.type,
                 cue_msg.sensitivity,
                 cue_msg.contrast,
                 cue_msg.theta
                 );

      result.setRelativeWeight(cue_msg.relative_weight);

      return result;
    }

    std::string toString(){
      std::stringstream ss;
      ss << "Type: " << this->type << "\n"
         << "Sens: " << this->sensitivity << "\n"
         << "Ctst: " << this->contrast << "\n"
         << "Thta: " << this->theta << "\n";
      return ss.str();
    }

    /**
       Translate a bb_util::Cue object into an equivalent
       bb_util::cue_msg format.
       \param cue The bb_util::Cue object.
       \return An equivalent bb_util::cue_msg.
    */
    static bb_util::cue_msg toMsg(Cue cue){
      bb_util::cue_msg msg;

      msg.type = cue.getType();
      msg.sensitivity = cue.getSensitivity();
      msg.contrast = cue.getContrast();
      msg.theta = cue.getTheta();
      msg.relative_weight = cue.getRelativeWeight();

      return msg;
    }


    /**
       Equality check. Two Cue objects are defined as equal if they
       are of the same type
    */
    bool operator== (Cue& rhs){
      bool eq = rhs.type.compare(type) ? false : true;
      return eq;
    }


    /**
       Assignment overload (creates a copy of rhs).
    */
    Cue operator= (Cue& rhs){
      type = rhs.getType();
      sensitivity = rhs.getSensitivity();
      contrast = rhs.getContrast();
      theta = rhs.getTheta();
      relative_weight = rhs.getRelativeWeight();
      Cue cue(type, sensitivity, contrast, theta);
      return cue;
    }
  };

}

#endif
