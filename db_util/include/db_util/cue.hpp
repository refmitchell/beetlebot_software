// Global utility class to represent a Cue;
// The same construction should be usable from cue detection to
// the central complex input.

#include "db_util.h"
#include "db_util/cue_msg.h"

namespace db_util{
  class Cue{
  private:
    std::string type =  "";
    double sensitivity = 0;
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

    //
    // Static methods for msg translation
    //
    static Cue toCue(db_util::cue_msg cue_msg){
      Cue result(cue_msg.type,
                 cue_msg.sensitivity,
                 cue_msg.reliability,
                 cue_msg.theta
                 );

      result.setRelativeWeight(cue_msg.relative_weight);

      return result;
    }

    static db_util::cue_msg toMsg(Cue cue){
      db_util::cue_msg msg;

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
      have the same type field.
    */
    bool operator== (Cue& rhs){
      return rhs.type.compare(type);
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
