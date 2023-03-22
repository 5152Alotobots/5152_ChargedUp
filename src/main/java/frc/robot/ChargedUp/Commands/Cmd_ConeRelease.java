// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.ChargedUp.Hand.SubSys_Hand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_ConeRelease extends SequentialCommandGroup {
  private SubSys_Arm subSys_Arm;
  private SubSys_Hand subSys_Hand;
  
  /** Creates a new Cmd_ConeRelease. */
  public Cmd_ConeRelease(SubSys_Arm subSys_Arm, SubSys_Hand subSys_Hand) {
    this.subSys_Arm = subSys_Arm;
    this.subSys_Hand = subSys_Hand;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Cmd_SubSys_Arm_JoysticDefault(subSys_Arm, ()-> -0.15, ()-> 0.0).withTimeout(0.5),
      new InstantCommand(subSys_Hand::CloseHand, subSys_Hand));
  }
}
