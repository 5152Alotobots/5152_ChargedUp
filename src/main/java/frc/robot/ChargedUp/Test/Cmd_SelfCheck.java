// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Pneumatics.SubSys_Pneumatics;

public class Cmd_SelfCheck extends CommandBase {
  /* SUBSYSTEMS */
  private final SubSys_Arm subSys_Arm;
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Hand subSys_Hand;
  private final SubSys_PigeonGyro subSys_PigeonGyro;
  private final SubSys_Pneumatics subSys_Pneumatics;

  /** Creates a new Cmd_SelfCheck. */
  public Cmd_SelfCheck(SubSys_Arm subSys_Arm, SubSys_DriveTrain subSys_DriveTrain, SubSys_Hand subSys_Hand, SubSys_PigeonGyro subSys_PigeonGyro, SubSys_Pneumatics subSys_Pneumatics) {
    /* Local subsystems */
    this.subSys_Arm = subSys_Arm;
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Hand = subSys_Hand;
    this.subSys_PigeonGyro = subSys_PigeonGyro;
    this.subSys_Pneumatics = subSys_Pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm, this.subSys_DriveTrain, this.subSys_Hand, this.subSys_PigeonGyro, this.subSys_Pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
