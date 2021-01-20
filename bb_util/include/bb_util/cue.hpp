// Global utility class to represent a Cue;
// The same construction should be usable from cue detection to
// the central complex input.

#include "bb_util.h"
#include "bb_util/cue_msg.h"

namespace bb_util{
  class Cue{
  private:
    std::string type =  "";
    double sensitivity = 1;
    double reliability = 0;
    double relative_weight = 0;
    double theta = 0;

  public:
    // Ctor: cues are constructed before their relative
    // weight is known, this is computed later.
    Cue(std::string type,
        double sensitivity,
        double reliability,
        double azimuth):
      type(type),
      sensitivity(sensitivity),
      reliability(reliability),
      theta(azimuth) {}

    // Getters
    double getReliability(){ return this->reliability; }
    std::string getType(){ return this->type; }
    double getRelativeWeight(){ return this->relative_weight; }
    double getTheta(){ return this->theta; }
    double getSensitivity(){ return this->sensitivity; }

    // Setters
    void setRelativeWeight(double rw){ relative_weight = rw; }

    void setReliability(double r){
      // If the attempted set is invalid, don't update the value.
      this->reliability = (r >= 0 && r <= 1) ? r : this->reliability;
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
                 cue_msg.reliability,
                 cue_msg.theta
                 );

      result.setRelativeWeight(cue_msg.relative_weight);

      return result;
    }

    static bb_util::cue_msg toMsg(Cue cue){
      bb_util::cue_msg msg;

      msg.type = cue.getType();
      msg.sensitivity = cue.getSensitivity();
      msg.reliability = cue.getReliability();
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
    bool operator= (Cue& rhs){
      type = rhs.getType();
      sensitivity = rhs.getSensitivity();
      reliability = rhs.getReliability();
      theta = rhs.getTheta();
      relative_weight = rhs.getRelativeWeight();
    }
  };

}
