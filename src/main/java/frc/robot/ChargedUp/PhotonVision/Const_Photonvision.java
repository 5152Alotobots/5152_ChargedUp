package frc.robot.ChargedUp.PhotonVision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Const_Photonvision {
    
    public final static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4);
    public final static double TARGET_HEIGHT_METERS = Units.inchesToMeters(9.5);
    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);
    public static final double IN_RANGE_AREA_PERCENT = 0.8; //TODO: MIGHT BE 80 INSTEAD OF 0.8

    public static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                    0, 0,
                    0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    public static class Cameras {
        public static final PhotonCamera frontCamera = new PhotonCamera("frontCamera");
        public static final PhotonCamera backCamera = new PhotonCamera("backCamera");
    }

    public static class Pipelines {
        public static final int Apriltag = 0;
        public static final int Cone = 1;
        public static final int Cube = 2;
    }
 
}