package frc.Common;

/** Create a target with the height and detection type */
public class Target {

    // These are the main 3 detection types I know of. More can be added if needed.
    public enum DetectionType {
        Reflective,
        AprilTag,
        GamePiece;
    }

    private String name;
    
    private double heightFromGroundsInInches;
    private DetectionType detectionType;
    private double alignmentTolerance;


    /**
     * Create a new target object
     * @param name
     * @param heightFromGroundsInInches
     * @param detectionType
     * @param alignmentTolerance in degrees (Lower is more precise)
     */
    public Target(String name, double heightFromGroundsInInches, DetectionType detectionType, double alignmentTolerance) {
        this.heightFromGroundsInInches = heightFromGroundsInInches;
        this.detectionType = detectionType;
    }
    
   /**
    * @return Height from ground in inches.
    */
   public double getHeightFromGroundsInInches() {
        return heightFromGroundsInInches;
    }

    /**
     * @return Return the targets detection type.
     */
    public DetectionType getDetectionType() {
        return detectionType;
    }

    /**
     * @return Return targets tolerance for alignment.
     */
    public double getAlignmentTolerance() {
        return alignmentTolerance;
    }
    
    /**
     * @return Return the name of target.
     */
    public String getName() {
        return name;
    }
}