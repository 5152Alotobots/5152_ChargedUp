// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_ArmDefault extends CommandBase {
  /** Creates a new Cmd_RotateArm. */
  private final SubSys_Arm armSubsys;

  private final DoubleSupplier rotateAxis;
  private final DoubleSupplier extendAxis;

  public Cmd_ArmDefault(SubSys_Arm armSubsys, DoubleSupplier rotateAxis, DoubleSupplier extendAxis) {
    this.armSubsys = armSubsys;
    this.rotateAxis = rotateAxis;
    this.extendAxis = extendAxis;

    addRequirements(armSubsys);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
    armSubsys.rotate_UntilOuterBoundary(rotateAxis.getAsDouble() * Const_Arm.kMAX_ROTATION_SPEED); 
    armSubsys.extend_UntilOuterBoundary(extendAxis.getAsDouble() * Const_Arm.kMAX_EXTENSION_SPEED); 
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
