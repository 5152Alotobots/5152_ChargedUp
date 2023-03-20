package frc.robot.ChargedUp.AprilTagPoseEstimation;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Const_AprilTagPoseEstimation {
    public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                    0, 0,
                    0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    public static final PhotonCamera aprilTagCamera = new PhotonCamera("aprilTagCamera");
}