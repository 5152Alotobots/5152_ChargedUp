// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Hand.SubSys_Hand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ConeDropAndRelease extends SequentialCommandGroup {
  /** Creates a new Cmd_ConeDropAndRelease. */
  public Cmd_ConeDropAndRelease(
    SubSys_Arm subSys_Arm,
    SubSys_Hand subSys_Hand,
    double armRotatePosCmd,
    double armExtensionPosCmd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Cmd_SubSys_Arm_PosCmd(subSys_Arm, armRotatePosCmd, true, armExtensionPosCmd, true),
      new InstantCommand(subSys_Hand::CloseHand, subSys_Hand)
    );
  }
}
