// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_HighLevelRotate extends CommandBase {
  /** Creates a new CubeAuto. */
  private final SubSys_Arm armSubsys;
  public Cmd_HighLevelRotate(SubSys_Arm armSubsys) {
    this.armSubsys = armSubsys;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 this.armSubsys.RotateArm(0,.8);
 System.out.println("Running Cmd");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.armSubsys.getShoulderRotation() >= 42 && this.armSubsys.getShoulderRotation() <= 47) {
      System.out.println("IN POS");
      return true;
    } 
    else{
      return false;
    }
  }
}
