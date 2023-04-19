// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_Retract_RevDeliveryPrePos;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_RevMidCubePlacement extends SequentialCommandGroup {
  private final SubSys_Arm subsysArm;
  private final SubSys_Hand subsysHand;

  /** Creates a new Cmd_HighConePlacement. */
  public Cmd_RevMidCubePlacement(SubSys_Arm subSys_Arm, SubSys_Hand subSys_Hand) {
    this.subsysArm = subSys_Arm;
    this.subsysHand = subSys_Hand;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new Cmd_SubSys_Arm_Retract_RevDeliveryPrePos(subsysArm),  // Move to Rev Delivery Preposition
      new Cmd_SubSys_Arm_PosCmd(subsysArm, -185.0, true, 1.54, true)
        .withTimeout(3), // Rotate and Extend to Mid Cube Deliver Position
      new WaitCommand(1), // Add Settling Time
      new InstantCommand(subsysHand::CloseHand, subsysHand), // Open hand (reversed)
      new Cmd_SubSys_Arm_PosCmd(subsysArm, -165.0, true, 1.54, true)
        .withTimeout(3)); // Rotate up to Clear Grid
  }
}
