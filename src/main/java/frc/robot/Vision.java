package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;


public  class Vision {
    //================== Cameras ========================
    public static PhotonCamera armCam = new PhotonCamera(VisionConstants.ARM_CAM_NAME);
    public static PhotonCamera intakeCam = new PhotonCamera(VisionConstants.INTAKE_CAM_NAME);

    //==================== Targets ==========================
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    //==================== Pose Estimation ==========================
    public static Transform3d robotToCam = new Transform3d(VisionConstants.ROBOT_TO_CAM_TRANSLATION, VisionConstants.ROBOT_TO_CAM_ROTATION); // In meters and radians
    public static Pose2d cameraPose = new Pose2d(VisionConstants.ROBOT_TO_CAM_TRANSLATION.toTranslation2d(), VisionConstants.ROBOT_TO_CAM_ROTATION.toRotation2d());

    public static double getDistanceTan(PhotonTrackedTarget target) {
        return (57.125 - 6) / Math.tan(Units.degreesToRadians(target.getPitch()) + Units.degreesToRadians(15));
    }

    public static double getAngleFromArea(double area) {
        double a = -274.818;
        double k = -1.51204;
        double d = -1.3563;
        double c = -49.8021;

       return a * (Math.pow(2, (k * (area - d)))) + c;
    }

    /**
     * Absolute VILE looking line, but pretty much get a list of targets from the result, then makes a stream for the list.
     * That stream is then filtered to only include those that don't equal 3 or 8 (The offset speaker tags)
     * Then grab the first element in the list, which is sorted based on the sorting mode (default is largest). So it returns the largest target that isn't 3 or 8.
     * And if you need to, you can chain multiple .filter()'s together
     * @param result
     * @return Largest Target not on the ignore list
     */
    public static PhotonTrackedTarget filterResults(PhotonPipelineResult result) {
        return result.getTargets().stream().filter(t -> t.getFiducialId() != 3 && t.getFiducialId() != 8).findFirst().get();
    }
}