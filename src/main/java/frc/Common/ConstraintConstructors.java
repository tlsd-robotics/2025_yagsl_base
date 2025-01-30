package frc.Common;

/** Create custom contraints here */
public class ConstraintConstructors {


    // Not sure how useful this one will be for future robots, but it is here if it is needed. 
    // In reality, this can be any value, not just pitch. So either rename it to something more generic or
    // just use it as is and know that is multipurpose
    public static class PitchConstraint {

        private final double UPPER_CONSTRAINT;
        private final double LOWER_CONSTRAINT;
    
        /**
         * @param LOWER_CONSTRAINT
         * @param UPPER_CONSTRAINT
         */
        public PitchConstraint(double LOWER_CONSTRAINT, double UPPER_CONSTRAINT) {
          this.LOWER_CONSTRAINT = LOWER_CONSTRAINT;
          this.UPPER_CONSTRAINT = UPPER_CONSTRAINT;
        }
    
        /**
         * @return Returns Lower Constraint of a pitch constraint
         */
        public double getLowerConstraint() {
            return LOWER_CONSTRAINT;
          }

        /**
         * @return Returns Upper Constraint of a pitch constraint
         */
        public double getUpperConstraint() {
          return UPPER_CONSTRAINT;
        }
      }
}
