package frc.Common;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.Common.ConstraintConstructors.PitchConstraint;
import frc.Common.SetpointConstructors.DoubleSolenoidGroupSetpoint;

/** A custom group class for Double Solenoids */

// This class is built out for the purpose of making solenoids easier to work with alongside implementing
// constraints since I know that was a pain to deal with for 2023. 
public class DoubleSolenoidGroup {
    private DoubleSolenoid solenoidOne;
    private DoubleSolenoid solenoidTwo;

    public DoubleSolenoidGroup(DoubleSolenoid solenoidOne, DoubleSolenoid solenoidTwo) {
        this.solenoidOne = solenoidOne;
        this.solenoidTwo = solenoidTwo;
    }

    /**
     * @return Returns Solenoid one
     */
    public DoubleSolenoid getSolenoidOne() {
        return solenoidOne;
    }

    /**
     * @return Returns Solenoid two
     */
    public DoubleSolenoid getSolenoidTwo() {
        return solenoidTwo;
    }

    /**
     * Sets both solenoids according to a single setpoint object.
     * @param setpoint DoubleSolenoidGroupSetpoint object
     */
    public void set(DoubleSolenoidGroupSetpoint setpoint) {
        solenoidOne.set(setpoint.getCylinderValueOne());
        solenoidTwo.set(setpoint.getCylinderValueTwo());
    }

    /**
     * Sets both solenoids according to a setpoint object. Will not trigger solenoids if current value is outside of the range provided by the constraint.
     * @param setpoint
     * @param currentValue
     * @param constraint
     */
    public void setWithConstraint(DoubleSolenoidGroupSetpoint setpoint, double currentValue, PitchConstraint constraint) {
        if (currentValue > constraint.getLowerConstraint() && currentValue < constraint.getUpperConstraint()) {
            solenoidOne.set(setpoint.getCylinderValueOne());
            solenoidTwo.set(setpoint.getCylinderValueTwo());
        }
        
    }

    /**
     * Sets both solenoids to the Off value. Meaning no air is pulling in or out. 
     */
    public void disable() {
        solenoidOne.set(Value.kOff);
        solenoidTwo.set(Value.kOff);
    }
}
