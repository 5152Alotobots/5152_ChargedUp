// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AprilTagPoseEstimation;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class SubSys_AprilTagPoseEstimation extends SubsystemBase {

  private PhotonPoseEstimator photonPoseEstimator;

  public SubSys_AprilTagPoseEstimation() {
    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      photonPoseEstimator =
              new PhotonPoseEstimator(
                      fieldLayout, PoseStrategy.MULTI_TAG_PNP, Const_AprilTagPoseEstimation.aprilTagCamera, Const_AprilTagPoseEstimation.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
  }
  }

  @Override
  public void periodic() {
    
  }
    
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
}

}

