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
  private final ProfiledPIDController xDistancePID;
  private double targetYDistance;
  private double initialYDistance;
  private final ProfiledPIDController yDistancePID;
  private double targetHeadingDegrees;
  private final ProfiledPIDController profiledRotationPID;
  private final TrapezoidProfile.Constraints profiledRotationConstraints;
  private final TrapezoidProfile.Constraints profiledMoveConstraints;

  public Cmd_SubSyst_DriveTrain_Drive4Distance(
    SubSys_DriveTrain subSys_DriveTrain,
    double targetXDistance,
    double targetYDistance) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.targetXDistance = targetXDistance;
    this.initialXDistance = 0.0;
    this.targetYDistance = targetYDistance;
    this.initialYDistance = 0.0;
    this.targetHeadingDegrees = 0.0;
    
    this.profiledMoveConstraints = new TrapezoidProfile.Constraints(
    SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxSpd,
     SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxAccel
     );

    this.xDistancePID = new ProfiledPIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain,
      this.profiledMoveConstraints);

    this.yDistancePID = new ProfiledPIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain,
      this.profiledMoveConstraints);

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
    this.xDistancePID.setTolerance(0.1);
    this.yDistancePID.setTolerance(0.1);
    this.profiledRotationPID.setIntegratorRange(-.3, 0.3);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    Pose2d currPose = this.subSys_DriveTrain.getPose();
    this.initialXDistance = currPose.getX();
    this.xDistancePID.reset(this.initialXDistance);
    this.xDistancePID.setGoal(this.targetXDistance);
    
    this.initialYDistance = currPose.getY();
    this.yDistancePID.reset(this.initialYDistance);
    this.yDistancePID.setGoal(this.targetYDistance);
    
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

    SmartDashboard.putNumber("xPID", xCmd);

    double yCmd = this.yDistancePID.calculate(
        currPose.getY()-this.initialYDistance);

        SmartDashboard.putNumber("yPID", yCmd);

    double rotCmd = this.profiledRotationPID.calculate(
      this.subSys_DriveTrain.getHeading().getDegrees(),
      this.targetHeadingDegrees);
    this.subSys_DriveTrain.Drive(
      xCmd,
      yCmd,
      rotCmd,
      false,
      false,
      false);


      SmartDashboard.putNumber("Drive4Dist_Goal X", this.xDistancePID.getGoal().position);
      SmartDashboard.putNumber("Drive4Dist_Setpoint_Position X", this.xDistancePID.getSetpoint().position);
      SmartDashboard.putNumber("Drive4Dist_SetPoint_Velocity X",this.xDistancePID.getSetpoint().velocity);
      SmartDashboard.putNumber("Drive4Dist_Error X", this.xDistancePID.getPositionError());
      
      //SmartDashboard.putNumber("Drive4Dist_rotPIDCmd", xCmd);
      //SmartDashboard.putNumber("Drive4Dist_rotFFCmd", rotFFCmd);
      //SmartDashboard.putNumber("Drive4Dist_rotCmd", rotCmd);
  
      //Test to see if finished
      //SmartDashboard.putBoolean("Drive4Dist Finished", this.profiledRotationPID.atGoal());
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
    if (this.xDistancePID.atGoal() &&
         this.yDistancePID.atGoal() &&
         this.profiledRotationPID.atGoal()){
      return true;
    }else{
      return false;
    }
  }
}
