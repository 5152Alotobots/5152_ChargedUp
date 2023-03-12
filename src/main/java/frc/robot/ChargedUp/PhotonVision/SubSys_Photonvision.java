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

public class SubSys_Photonvision extends SubsystemBase {
  /** Creates a new PhotonVisionSubsytem. */
  public SubSys_Photonvision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /* XY PID */
    public PIDController XYcontroller = new PIDController(DriveTrainTrajSettings.DriveTrajectoryPID.Pgain, DriveTrainTrajSettings.DriveTrajectoryPID.Igain, DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);

    /* Z PID */
    public PIDController Zcontroller = new PIDController(DriveTrainTrajSettings.RotationTrajectoryPID.Pgain, DriveTrainTrajSettings.RotationTrajectoryPID.Igain, DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);


    /** VISION */
    public double getRangeToTarget(PhotonPipelineResult result){

      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
            Const_Photonvision.CAMERA_HEIGHT_METERS,
            Const_Photonvision.TARGET_HEIGHT_METERS,
            Const_Photonvision.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
      return range;
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
        return XYcontroller.calculate(getRangeToTarget(result), Const_Photonvision.GOAL_RANGE_METERS);
      } else {
        return 0;
      }
    }

    /** Calculate strafe speed */
    public double getVisionStrafeSpeed(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return XYcontroller.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        return 0;
      }
    }

    /** Returns true if the PID controllers don't need to move any further/you are at the target */
    public boolean isAtTarget(PhotonPipelineResult result){
      if (result.hasTargets()) {
        return XYcontroller.atSetpoint() && Zcontroller.atSetpoint();
      } else {
        return false;
      }
    }
  
    
}

