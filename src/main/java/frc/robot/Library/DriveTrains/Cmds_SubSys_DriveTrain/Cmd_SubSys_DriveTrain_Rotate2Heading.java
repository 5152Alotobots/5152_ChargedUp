// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants;

public class Cmd_SubSys_DriveTrain_Rotate2Heading extends CommandBase {
  /** Creates a new Cmd_SubSys_DriveTrain_Rotate2Heading. */

  private final SubSys_DriveTrain subSys_DriveTrain;
  private final double targetHeadingDegrees;
  private final ProfiledPIDController profiledRotationPID;
  private final TrapezoidProfile.Constraints profiledRotationConstraints;

  public Cmd_SubSys_DriveTrain_Rotate2Heading(
    SubSys_DriveTrain subSys_DriveTrain,
    double targetHeadingDegrees) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.targetHeadingDegrees = targetHeadingDegrees;

    this.profiledRotationConstraints = new TrapezoidProfile.Constraints(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxRotSpeed,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxRotAccel);
    this.profiledRotationPID = new ProfiledPIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Dgain,
      this.profiledRotationConstraints);

    this.profiledRotationPID.enableContinuousInput(-180, 180);
    this.profiledRotationPID.setTolerance(1, 1);
    this.profiledRotationPID.setIntegratorRange(-.3, 0.3);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.profiledRotationPID.reset(this.subSys_DriveTrain.getHeading().getDegrees());
    this.profiledRotationPID.setGoal(this.targetHeadingDegrees);

    //log target to dashboard
    SmartDashboard.putData(profiledRotationPID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotCmd = this.profiledRotationPID.calculate(
      this.subSys_DriveTrain.getHeading().getDegrees(),
      this.targetHeadingDegrees);
    this.subSys_DriveTrain.Drive(
      0,
      0,
      rotCmd,
      false,
      false,
      false);

    //Log rotation command to dash
    SmartDashboard.putNumber("Rotate2Heading Rotation Speed", rotCmd);

    //Test to see if finished
    SmartDashboard.putBoolean("Rotate2Heading Finished", this.profiledRotationPID.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_DriveTrain.Drive(
      0.0,
      0.0,
      0.0,
      false,
      false,
      false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if (this.profiledRotationPID.atSetpoint()){
      return true;
    }else{
      return false;
    }
    */
    return false;
  }
}
