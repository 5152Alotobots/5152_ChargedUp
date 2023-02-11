// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants;

public class Cmd_SubSyst_DriveTrain_Drive4Distance extends CommandBase {
  /** Creates a new Cmd_SubSyst_DriveTrain_Drive4Distance. */

  private final SubSys_DriveTrain subSys_DriveTrain;
  private double targetXDistance;
  private double initialXDistance;
  private final PIDController xDistancePID;
  private double targetYDistance;
  private double initialYDistance;
  private final PIDController yDistancePID;
  private double targetHeadingDegrees;
  private final ProfiledPIDController profiledRotationPID;
  private final TrapezoidProfile.Constraints profiledRotationConstraints;

  public Cmd_SubSyst_DriveTrain_Drive4Distance(
    SubSys_DriveTrain subSys_DriveTrain,
    double targetXDistance,
    double targetYDistance) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.targetXDistance = targetXDistance;
    this.initialXDistance = 0.0;
    this.targetYDistance = targetYDistance;
    this.initialYDistance = 0.0;
    this.targetHeadingDegrees =0.0;
    
    this.xDistancePID = new PIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);

    this.xDistancePID.setTolerance(0.01);

    this.yDistancePID = new PIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);

    this.xDistancePID.setTolerance(0.01);

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
    this.subSys_DriveTrain.setPose(new Pose2d());
    this.subSys_DriveTrain.setGyroYawToZero();
    Pose2d currPose = this.subSys_DriveTrain.getPose();
    this.initialXDistance = currPose.getX();
    this.xDistancePID.reset();
    this.xDistancePID.setSetpoint(this.targetXDistance);
    this.xDistancePID.setTolerance(0.01);
    
    this.initialYDistance = currPose.getY();
    this.yDistancePID.reset();
    this.yDistancePID.setSetpoint(this.targetYDistance);
    this.yDistancePID.setTolerance(0.01);
    
    this.targetHeadingDegrees = this.subSys_DriveTrain.getHeading().getDegrees();
    this.profiledRotationPID.reset(this.subSys_DriveTrain.getHeading().getDegrees());
    this.profiledRotationPID.setGoal(this.targetHeadingDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = this.subSys_DriveTrain.getPose();
    
    double xCmd = this.xDistancePID.calculate(
      currPose.getX()-this.initialXDistance);

    double yCmd = this.yDistancePID.calculate(
        currPose.getY()-this.initialYDistance);

    double rotCmd = this.profiledRotationPID.calculate(
      this.subSys_DriveTrain.getHeading().getDegrees(),
      this.targetHeadingDegrees);
    this.subSys_DriveTrain.Drive(
      xCmd,
      yCmd,
      0, //rotCmd,
      false,
      false,
      false);


      SmartDashboard.putNumber("Drive4Distance_xSetpoint", this.xDistancePID.getSetpoint());
      SmartDashboard.putNumber("Drive4Distance_xError", this.xDistancePID.getPositionError());
      SmartDashboard.putNumber("Drive4Distance_xCmd", xCmd);
      SmartDashboard.putBoolean("Drive4Distance_AtSetpoint", this.xDistancePID.atSetpoint());
      //SmartDashboard.putNumber("Rotate2Heading_Setpoint_Position", this.profiledRotationPID.getSetpoint().position);
      //SmartDashboard.putNumber("Rotate2Heading_SetPoint_Velocity",this.profiledRotationPID.getSetpoint().velocity);
      //SmartDashboard.putNumber("Rotate2Heading_Error", this.profiledRotationPID.getPositionError());
      
      //SmartDashboard.putNumber("Rotate2Heading_rotPIDCmd", rotPIDCmd);
      //SmartDashboard.putNumber("Rotate2Heading_rotFFCmd", rotFFCmd);
      //SmartDashboard.putNumber("Rotate2Heading_rotCmd", rotCmd);
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
    if (this.xDistancePID.atSetpoint()){
      return true;
    }else{
      return false;
    }
  }
}
