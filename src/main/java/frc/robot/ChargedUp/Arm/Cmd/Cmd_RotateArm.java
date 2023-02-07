// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;

public class Cmd_RotateArm extends CommandBase {
  /** Creates a new Cmd_RotateArm. */
  private final SubSys_Arm ArmSubsys;
  private final SubSys_DriverStation DriverStationSubsys;
  public Cmd_RotateArm(SubSys_Arm ArmSubsys, SubSys_DriverStation DriverStationSubSys) {
    this.ArmSubsys = ArmSubsys;
    this.DriverStationSubsys = DriverStationSubSys;

    addRequirements(ArmSubsys, DriverStationSubSys);
  }
  public double CurrentAngle; 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStationSubsys.GetArmAxis();
    
    


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
