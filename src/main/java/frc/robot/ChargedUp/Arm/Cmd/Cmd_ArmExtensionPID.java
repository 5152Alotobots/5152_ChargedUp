// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants;

public class Cmd_ArmExtensionPID extends CommandBase {
  /** Creates a new Cmd_SubSyst_DriveTrain_Drive4Distance. */
  private final SubSys_Arm subSys_Arm;

  private double targetPosition;
  private double initialPosition;
  private final PIDController distancePID;

  private final SimpleMotorFeedforward feedForward;

  public Cmd_ArmExtensionPID(
      SubSys_Arm subSys_Arm,
      double targetPositionCM) {

    this.subSys_Arm = subSys_Arm;
    this.targetPosition = targetPositionCM;
    this.initialPosition = 0.0;

    this.feedForward =
        new SimpleMotorFeedforward(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kS,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kV);

    this.distancePID =
        new PIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
    this.distancePID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.VelocityTolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.subSys_DriveTrain.setPose(new Pose2d());
    double currPose = this.subSys_Arm.getArmExtension(); //TODO: TEST
    this.initialPosition = currPose;
    this.distancePID.reset();
    this.distancePID.setSetpoint(this.targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double currPose = this.subSys_Arm.getArmExtension(); //TODO: TEST 

    double mvCmd =
        this.distancePID.calculate(currPose - this.initialPosition, this.targetPosition);

    SmartDashboard.putNumber("PID", mvCmd);
    this.subSys_Arm.armExtentionMinMax(mvCmd, 0, 60);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_Arm.armExtentionMinMax(0, 0, 60);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* PID */
    if (this.distancePID.atSetpoint()) {
      return true;
    } else {
      return false;
    }
    /* ProfiledPID
    if (this.xDistancePID.atGoal() &&
         this.yDistancePID.atGoal() &&
         this.profiledRotationPID.atGoal()){
      return true;
    }else{
      return false;
    }
    */
  }
}
