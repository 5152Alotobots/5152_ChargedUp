package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_ArmExtensionPID extends PIDCommand {
  /** Creates a new Cmd_ArmExtensionPID. */

  private final SubSys_Arm subSys_Arm;
  private final double targetPositionCM;
  private double initialPosition;

  public Cmd_ArmExtensionPID(SubSys_Arm subSys_Arm, double targetPositionCM) {
    //Use super to use the parent class of PIDCommand
    super(
      new PIDController(Const_Arm.ExtendPID.kP, Const_Arm.ExtendPID.kI, Const_Arm.ExtendPID.kD),
      () -> subSys_Arm.getExtendLength(),
      () -> targetPositionCM,
      output -> subSys_Arm.ExtendArm(0, output),
      subSys_Arm
    );
    this.subSys_Arm = subSys_Arm;
    this.targetPositionCM = targetPositionCM;

    getController().setTolerance(Const_Arm.ExtendPID.kTOLERANCE, Const_Arm.ExtendPID.kSPEED_TOLERANCE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSys_Arm.ExtendArm(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* PID */
    if (getController().atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}