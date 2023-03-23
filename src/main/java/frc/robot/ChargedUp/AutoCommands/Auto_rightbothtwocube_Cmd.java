// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

/**
 * *Link For PathPlaner *
 * https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e65ac68f1d_0_48
 */
public class Auto_rightbothtwocube_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Arm m_Arm;
  private final SubSys_Hand m_Hand;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_rightbothtwocube_Cmd(
      SubSys_DriveTrain driveSubSys,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Arm subsysArm,
      SubSys_Hand subsysHand) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_Arm = subsysArm;
    m_Hand = subsysHand;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new Cmd_whatever the arm one is
        // Hand is reversed
        new Cmd_SubSys_Arm_PosCmd(subsysArm, -147.0, true, 1.54, true).withTimeout(4),
        new WaitCommand(2.5),
        new InstantCommand(subsysHand::CloseHand, subsysHand),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new Cmd_SubSys_Arm_PosCmd(subsysArm, 42.0, false, 0.8, true).withTimeout(4),
                new Cmd_SubSys_Arm_PosCmd(subsysArm, 42.0, true, 0.8, false).withTimeout(4),
                new Cmd_SubSys_Arm_PosCmd(subsysArm, 42.0, false, 1.0, true).withTimeout(4)),
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "rightbothtwocube1", true, true)),
        new InstantCommand(subsysHand::OpenHand, subsysHand),
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "rightbothtwocube2", false, false),
            new Cmd_SubSys_Arm_PosCmd(subsysArm, 10.0, true, 0.8, true).withTimeout(4)),
        new Cmd_AutoBalance(pigeonGyro, driveSubSys));
  }
}