// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SubSys_Arm_Retract_RevDeliveryPrePos extends SequentialCommandGroup {
  private final SubSys_Arm subsysArm;

  /** Creates a new Cmd_HighConePlacement. */
  public Cmd_SubSys_Arm_Retract_RevDeliveryPrePos(SubSys_Arm subSys_Arm) {
    this.subsysArm = subSys_Arm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Cmd_SubSys_Arm_PosCmd(subsysArm, subSys_Arm.getArmShoulderAngle().getDegrees(), false, 0.85, false)
        .withTimeout(1.5), // Retract Arm
      new Cmd_SubSys_Arm_PosCmd(subsysArm, -120.0, true, 0.85, true)
        .withTimeout(4)); // Lift arm to high position
  }
}
