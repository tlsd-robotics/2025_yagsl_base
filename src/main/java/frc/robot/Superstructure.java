package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.SuperstructureConstants;


public final class Superstructure {
    private static final PowerDistribution pdp = new PowerDistribution(SuperstructureConstants.PDP_ID, ModuleType.kRev);

    public static void start() {
        pdp.clearStickyFaults();
    }

    public static PowerDistribution getPDP() {
        return pdp;
    }
}
