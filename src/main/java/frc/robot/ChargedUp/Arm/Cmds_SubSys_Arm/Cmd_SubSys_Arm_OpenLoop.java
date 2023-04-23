// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.Constants.Robot;
import frc.robot.Library.DriverStation.JoystickUtilities;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_OpenLoop extends CommandBase {
  /** Creates a new Cmd_SubSys_Arm_JoysticDefault. */
  SubSys_Arm subSys_Arm;

  private final Double armRotate;
  private final Double armExtend;

  public Cmd_SubSys_Arm_OpenLoop(
      SubSys_Arm subSys_Arm, Double armRotate, Double armExtend) {

    this.subSys_Arm = subSys_Arm;
    this.armRotate = armRotate;
    this.armExtend = armExtend;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subSys_Arm.setArmCmd(
        JoystickUtilities.joyDeadBndSqrdScaled(
            this.armRotate, 0.05, Robot.Calibrations.Arm.ArmMaxRotSpd),
        JoystickUtilities.joyDeadBndSqrdScaled(
            this.armExtend, 0.05, Robot.Calibrations.Arm.ArmExtensionMaxSpd));
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
