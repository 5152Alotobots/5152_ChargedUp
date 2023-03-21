// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.PhotonVision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class SubSys_Photonvision extends SubsystemBase {
  /** Creates a new PhotonVisionSubsytem. */
  private final SubSys_Arm m_Arm;
  public SubSys_Photonvision(SubSys_Arm armSubSys) {
    m_Arm = armSubSys;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /* X PID */
    public PIDController Xcontroller = new PIDController(DriveTrainTrajSettings.DriveTrajectoryPID.Pgain, DriveTrainTrajSettings.DriveTrajectoryPID.Igain, DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);

    /* Y PID */
    public PIDController Ycontroller = new PIDController(DriveTrainTrajSettings.DriveTrajectoryPID.Pgain, DriveTrainTrajSettings.DriveTrajectoryPID.Igain, DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);

    /* Z PID */
    
    public PIDController Zcontroller = new PIDController(DriveTrainTrajSettings.RotationTrajectoryPID.Pgain, DriveTrainTrajSettings.RotationTrajectoryPID.Igain, DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);

    /* VISION */
    /** Calculate distance to target */
    public double getRangeToTarget(PhotonPipelineResult result, Boolean UseArmCalculatedAngle){
      if (UseArmCalculatedAngle){
      return
        PhotonUtils.calculateDistanceToTargetMeters(
            m_Arm.getCameraHeight(),
            Const_Photonvision.TARGET_HEIGHT_METERS,
            m_Arm.getShoulderRotationRadians() - 90,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
      } else {
        return
        PhotonUtils.calculateDistanceToTargetMeters(
            Const_Photonvision.CAMERA_HEIGHT_METERS,
            Const_Photonvision.TARGET_HEIGHT_METERS,
            Const_Photonvision.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));
      }
    }

    /** Calculate rotation speed */
    public double getVisionRotSpeed(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return Zcontroller.calculate(result.getBestTarget().getSkew(), 0);
      } else {
        return 0;
      }
    }
    
    /** Calculate forward speed */
    public double getVisionForwardSpeed(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return Xcontroller.calculate(getRangeToTarget(result, false), Const_Photonvision.GOAL_RANGE_METERS);
      } else {
        return 0;
      }
    }

    /** Calculate strafe speed */
    public double getVisionStrafeSpeed(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return Ycontroller.calculate((result.getBestTarget().getYaw()) - Const_Photonvision.robotToCam.getY(), 0);
      } else {
        return 0;
      }
    }

    /** Returns true if the PID controllers don't need to move any further/you are at the target */
    public boolean isAtTarget(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return Xcontroller.atSetpoint() && Ycontroller.atSetpoint() && Zcontroller.atSetpoint();
      } else {
        return false;
      }
    }
  
    /** Returns true if the camera's target is taking up the acceptable percentage of the viewport
     * @param acceptablePercentage The percentage of the viewport the target should take up 0-1
     * @param result The result of the pipeline
     * @return True if the target is taking up the acceptable percentage of the viewport
     */
    public boolean isInRange(PhotonPipelineResult result, double acceptablePercentage){
      if (result.hasTargets()) {
        return result.getBestTarget().getArea() >= acceptablePercentage;
      } else {
        return false;
      }
    }

    
}

