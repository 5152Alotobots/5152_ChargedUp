package frc.robot.ChargedUp.PhotonVision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Const_Photonvision {
    // Camera constants
    public final static double CAMERA_HEIGHT_METERS = 0.47;
    public final static double CAMERA_TO_FRONT_ROBOT_METERS = 0.28;
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-28);

    // FORWARD: +, RIGHT: +, UP: + (USED FOR APRILTAGS)
    public static final Transform3d robotToCam =
    new Transform3d(
            new Translation3d(0.2, 0.105, 0.47),
            new Rotation3d(
                    0, -28,
                    0)); // Cam mounted facing forward, 28deg pitched DOWN, 20cm forward of center, 10.5cm RIGHT of center, 47cm UP from center.


    public static class PIDspeeds {
        public static final double Max_X_PID_Speed = 0.5;
        public static final double Max_Y_PID_Speed = 0.5;
        public static final double Max_Z_PID_Speed = 0.5;
    }

    public static class AcceptablePIDError {
        public static final double X_PID_Error = 0.04;
        public static final double Y_PID_Error = 0.1;
        public static final double Z_PID_Error = 1.1;
    }
    public static class Cameras {
        public static final PhotonCamera frontCamera = new PhotonCamera("frontCamera");
        public static final PhotonCamera backCamera = new PhotonCamera("backCamera");
    }

    public static class Pipelines {
        public static final int Apriltag = 2;
        public static final int Cone = 1;
        public static final int Cube = 0;
    }
 
    public static class TargetingConstants {
        public static class GridApriltag {
            // Target
            public final static double TARGET_HEIGHT_METERS = Units.inchesToMeters(15.13);
            public static final double GOAL_RANGE_METERS = 0.7;
            public static final double IN_RANGE_AREA_PERCENT = 0.8;
        }
        public static class Cone {
            // Target
            public final static double TARGET_HEIGHT_METERS = Units.inchesToMeters(5);
            public static final double GOAL_RANGE_METERS = 0.45;
            public static final double IN_RANGE_AREA_PERCENT = 0.8;
        }
        public static class Cube {
            // Target
            public final static double TARGET_HEIGHT_METERS = Units.inchesToMeters(4.5);
            public static final double GOAL_RANGE_METERS = 0.45;
            public static final double IN_RANGE_AREA_PERCENT = 0.8;
        }
    }
}