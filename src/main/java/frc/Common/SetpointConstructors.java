package frc.Common;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.Common.ConstraintClasses.RangeConstraint;

/** This is a collection of all setpoints for each subsystem */
public class SetpointConstructors {

    public static class PitchSetpoint {

        private final double angle;
        private final double tolerance;
    
        /**
         * Create new pitch setpoint. In reality it can be used for anything, not just pitch. 
         * @param angleDegrees
         * @param toleranceDegrees
         */
        public PitchSetpoint(double angleDegrees, double toleranceDegrees) {
          this.angle = angleDegrees;
          this.tolerance = toleranceDegrees;
        }
    
        /**
         * @return Returns the setpoint angle in degrees.
         */
        public double getSetpointAngleDegrees() {
          return angle;
        }
        
        /**
         * @return Returns the tolerance in degrees.
         */
        public double getToleranceDegrees() {
          return tolerance;
        }
    }

    public static class DoubleSolenoidGroupSetpoint {

      Value cylinderValueOne;
      Value cylinderValueTwo;
      RangeConstraint constraint;

      /**
       * Stores 2 solenoid values at once for use with the doublesolenoidgroup object's set function
       * @param forCylinderOne
       * @param forCylinderTwo
       */
      public DoubleSolenoidGroupSetpoint(Value forCylinderOne, Value forCylinderTwo) {
        this.cylinderValueOne = forCylinderOne;
        this.cylinderValueTwo = forCylinderTwo;
      }
      
      /**
       * @return Returns the value for solenoid one
       */
      public Value getCylinderValueOne() {
        return cylinderValueOne;
      }

      /**
       * @return Returns the value for solenoid two
       */
      public Value getCylinderValueTwo() {
        return cylinderValueTwo;
      }
    }



}
