// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_SubSys_Arm_IdlePositionHold extends CommandBase {
  /** Creates a new Cmd_SubSys_Arm_JoysticDefault. */
  SubSys_Arm subSys_Arm;

  private double posToHold;

  public Cmd_SubSys_Arm_IdlePositionHold(SubSys_Arm subSys_Arm) {

    this.subSys_Arm = subSys_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    posToHold = subSys_Arm.getArmShoulderAngle().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subSys_Arm.setArmPosCmd(posToHold, true, 0, false);
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
