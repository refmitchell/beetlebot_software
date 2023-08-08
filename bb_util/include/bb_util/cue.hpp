// Global utility class to represent a Cue;
// The same construction should be usable from cue detection to
// the central complex input.

#ifndef BBUTIL_CUE
#define BBUTIL_CUE

#include "bb_util.h"
#include "bb_util/cue_msg.h"

#include <sstream>

namespace bb_util{
  class Cue{
  private:
    std::string type =  "";
    double sensitivity = 1;
    double contrast = 0;
    double relative_weight = 0;
    double theta = 0;

  public:
    // Ctor: cues are constructed before their relative
    // weight is known, this is computed later.
    Cue(std::string type,
        double sensitivity,
        double contrast,
        double azimuth):
      type(type),
      sensitivity(sensitivity),
      contrast(contrast),
      theta(azimuth) {}

    // Getters
    double getContrast(){ return this->contrast; }
    std::string getType(){ return this->type; }
    double getRelativeWeight(){ return this->relative_weight; }
    double getTheta(){ return this->theta; }
    double getSensitivity(){ return this->sensitivity; }

    // Setters
    void setRelativeWeight(double rw){ relative_weight = rw; }

    void setContrast(double r){
      // If the attempted set is invalid, don't update the value.
      this->contrast = (r >= 0 && r <= 1) ? r : this->contrast;
    }

    void setAzimuth(double t){
      // Assume valid value in radians.
      this->theta = t;
    }

    //
    // Static methods for msg translation
    //
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
         << "Rlby: " << this->contrast << "\n"
         << "Thta: " << this->theta << "\n";
      return ss.str();
    }

    static bb_util::cue_msg toMsg(Cue cue){
      bb_util::cue_msg msg;

      msg.type = cue.getType();
      msg.sensitivity = cue.getSensitivity();
      msg.contrast = cue.getContrast();
      msg.theta = cue.getTheta();
      msg.relative_weight = cue.getRelativeWeight();

      return msg;
    }

    //
    // Operator overloads
    //

    /*
      For now we define two cues as equal if they
      have the same type field. Simple justification is that we have no evidence
      to suggest that the beetles use multiple cues within the same modality.
      Multiple light cues tend to confuse them, and multiple wind cues hasn't
      been tested
    */
    bool operator== (Cue& rhs){
      bool eq = rhs.type.compare(type) ? false : true;
      return eq;
    }


    /*
      Adding assignment for ease
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
