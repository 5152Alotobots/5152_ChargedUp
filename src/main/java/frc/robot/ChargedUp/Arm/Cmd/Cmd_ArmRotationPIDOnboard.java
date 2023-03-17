package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_ArmRotationPIDOnboard extends CommandBase {
  private final SubSys_Arm subSys_Arm;


  public Cmd_ArmRotationPIDOnboard(SubSys_Arm subSys_Arm, double targetHeadingDegrees) {

    this.subSys_Arm = subSys_Arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subSys_Arm.armRotationMoveToPos();
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
