/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.DriveTrains;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChargedUp.AprilTagPoseEstimation.SubSys_AprilTagPoseEstimation;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_SwerveDrive;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

public class SubSys_DriveTrain extends SubsystemBase {
  /** Creates a new Drive SubSystem. */

  // Drive Types - Select only 1

  // Tank
  // private final TankDriveSubSys m_Drive =
  // new TankDriveSubSys;

  // Mecanum
  // private final MecanumDriveSubSys m_Drive =
  // new MecanumDriveSubSys;

  // Swerve
  private SubSys_SwerveDrive driveTrain;

  // Drive Commands
  private double driveXDirCmd = 0;
  private double driveYDirCmd = 0;
  private double driveZRotCmd = 0;
  private boolean driveFieldOriented = true;
  private boolean driveRotateLeftPtCmd = false;
  private boolean driveRotateRightPtCmd = false;

  // GyroScope
  private SubSys_PigeonGyro gyroSubSys;

  // Apriltags
  private SubSys_AprilTagPoseEstimation aprilTagPoseEstimation;

  /**
   * SubSys_DriveTrain Constructor
   *
   * @param gyroSubSys SubSys_PigeonGyro
   * @param aprilTagPoseEstimation SubSys_AprilTagPoseEstimation
   */
  public SubSys_DriveTrain(SubSys_PigeonGyro gyroSubSys) {
    this.gyroSubSys = gyroSubSys;
    this.aprilTagPoseEstimation = new SubSys_AprilTagPoseEstimation();
    this.driveTrain = new SubSys_SwerveDrive(this.gyroSubSys);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Send Drive Commands
    driveTrain.drive(
        new Translation2d(driveXDirCmd, driveYDirCmd),
        driveZRotCmd,
        driveFieldOriented,
        true,
        driveRotateLeftPtCmd,
        driveRotateRightPtCmd);

    // Put pose and Heading to smart dashboard
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y", getPose().getY());
    SmartDashboard.putNumber("Heading (Degrees)", getHeading().getDegrees());

    // Put Drive Commands to smart dashboard
    SmartDashboard.putNumber("driveXDirCmd", driveXDirCmd);
    SmartDashboard.putNumber("driveYDirCmd", driveYDirCmd);
    SmartDashboard.putNumber("driveZRotCmd", driveZRotCmd);
  }

  /***********************************************************************************/
  /* ***** Public DriveTrain Methods *****                                               */
  /***********************************************************************************/

  // ***** DriveTrain Info *****

  /**
   * getMaxDriveSubSysSpd Returns Max Drive SubSystem Speed
   *
   * @return double DriveTrain Maximum Speed (m/s)
   */
  public double getMaxDriveSubSysSpd() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxSpd;
  }

  public double getMaxDriveTurboSubSysSpd() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboSpd;
  }

  public double getMaxDriveSubSysTurboPctOutput() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboPctOutput;
  }

  public double getMaxDriveSubSysTurboAccel() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboAccel;
  }


  /**
   * getMaxDriveSubSysRotSpd Returns Max Drive Subsystem Rotation
   *
   * @return double DriveTrain Maximum Speed (rads/s)
   */
  public double getMaxDriveSubSysRotSpd() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed;
  }

  public double getMaxDriveSubSysTurboRotSpd() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotSpeed;
  }

  public double getMaxDriveSubSysTurboRotPctOutput() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotPctOutput;
  }

  public double getMaxDriveSubSysTurboRotAccel() {
    return SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotAccel;
  }

  // ***** Drive Methods *****

  /**
   * Drive Method to drive the robot using setpoint.
   *
   * @param xSpdCmd Speed Cmd of the robot in the x direction (forward) m/s.
   * @param ySpdCmd Speed Cmd of the robot in the y direction (sideways) m/s.
   * @param rotSpdCmd Rotational speed Cmd of the robot. rad/s
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rotateLeftPt boolean Rotate around Left Pt
   * @param rotateRightPt boolean Rotate around Right Pt
   */
  @SuppressWarnings("ParameterName")
  public void Drive(
      double xSpdCmd,
      double ySpdCmd,
      double rotSpdCmd,
      boolean fieldRelative,
      boolean rotateLeftPtCmd,
      boolean rotateRightPtCmd) {

    // Limit Cmds to Chassis Limits
    driveXDirCmd =
        Math.min(
            Math.max(xSpdCmd, -SubSys_DriveTrain_Constants.DriveTrainMaxSpd),
            SubSys_DriveTrain_Constants.DriveTrainMaxSpd);
    driveYDirCmd =
        Math.min(
            Math.max(ySpdCmd, -SubSys_DriveTrain_Constants.DriveTrainMaxSpd),
            SubSys_DriveTrain_Constants.DriveTrainMaxSpd);
    driveZRotCmd =
        Math.min(
            Math.max(rotSpdCmd, -SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed),
            SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed);
    driveFieldOriented = fieldRelative;
    driveRotateLeftPtCmd = rotateLeftPtCmd;
    driveRotateRightPtCmd = rotateRightPtCmd;
  }

  // ***** Odometry *****

  /**
   * getHeading Get Swerve Drive Heading in Rotation2d
   *
   * @return Rotation2d Heading of the drive train
   */
  public Rotation2d getHeading() {
    return this.driveTrain.getHeading();
  }

  /**
   * setGyroYaw set Gyro Yaw Value
   *
   * @param degrees
   */
  public void setYaw(double degrees) {
    this.driveTrain.setYaw(degrees);
  }

  /** setGyroYawToZero set Gyro Yaw Value to Zero */
  public void setYawToZero() {
    this.driveTrain.setYawToZero();
  }

  /**
   * getPose Get the X and Y position of the drivetrain from the DriveTrain
   *
   * @return Pose2d X and Y position of the drivetrain
   */
  public Pose2d getPose() {
    return this.driveTrain.getPose();
  }

  /**
   * setPose Set Pose of the drivetrain
   *
   * @param pose Pose2d X and Y position of the drivetrain
   */
  public void setPose(Pose2d pose) {
    this.driveTrain.setPose(pose);
  }

  /**
   * setPoseToOrigin Set Pose of the drivetrain to 0,0
   *
   * @param pose Pose2d X and Y position of the drivetrain
   */
  public void setPoseToOrigin() {
    this.driveTrain.setPose(new Pose2d());
  }

  /** setPoseDriveMtrsToZero Set Pose and Drive Motors to Zero */
  public void setPoseDriveMtrsToZero() {
    this.driveTrain.setPoseDriveMtrsToZero();
  }

  // PhotonVision
  public void setPoseToAverageVisionAndOdometryEstimate() {
    // Get the estimated pose from the AprilTagPoseEstimator 
    Optional<EstimatedRobotPose> result = this.aprilTagPoseEstimation.getEstimatedGlobalPose(getPose());

    // If the pose is valid, average it with the current pose and set the new pose
    if (result.isPresent()) {
      double avgX = (getPose().getTranslation().getX() + result.get().estimatedPose.toPose2d().getTranslation().getX()) / 2;
      double avgY = (getPose().getTranslation().getY() + result.get().estimatedPose.toPose2d().getTranslation().getY()) / 2;
      Rotation2d avgRotation = getPose().getRotation().plus(result.get().estimatedPose.toPose2d().getRotation()).div(2);
  
      Pose2d avgPose = new Pose2d(avgX, avgY, avgRotation);

      this.driveTrain.setPose(avgPose);
    }
  }

  public void setPoseToVisionEstimate() {
    // Get the estimated pose from the AprilTagPoseEstimator 
    Optional<EstimatedRobotPose> result = this.aprilTagPoseEstimation.getEstimatedGlobalPose(getPose());

    // If the pose is valid, average it with the current pose and set the new pose
    if (result.isPresent()) {
      this.driveTrain.setPose(result.get().estimatedPose.toPose2d());
    }
  }
}
