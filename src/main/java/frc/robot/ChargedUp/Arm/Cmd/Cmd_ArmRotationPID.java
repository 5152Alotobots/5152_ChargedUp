package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;

public class Cmd_ArmRotationPID extends CommandBase {
  /** Creates a new Cmd_SubSys_DriveTrain_Rotate2Heading. */
  private final SubSys_Arm subSys_Arm;

  private final double targetHeadingDegrees;

  public Cmd_ArmRotationPID(SubSys_Arm subSys_Arm, double targetHeadingDegrees) {

    this.subSys_Arm = subSys_Arm;
    this.targetHeadingDegrees = targetHeadingDegrees;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.profiledRotationPID.reset(this.subSys_Arm.getShoulderRotation());
    // this.profiledRotationPID.setGoal(this.targetHeadingDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //rotate arm
    // this.subSys_Arm.rotateArmMinMax(rotCmd, -43, 80);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_Arm.rotateArmMinMax(0, -43, 35);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
