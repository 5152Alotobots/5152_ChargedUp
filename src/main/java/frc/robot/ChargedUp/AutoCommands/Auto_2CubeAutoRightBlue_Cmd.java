// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelExtend;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelRotate;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.Arm.SubSys_Arm;


public class Auto_2CubeAutoRightBlue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Hand m_hand;
  private final SubSys_Arm m_Arm;
  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_2CubeAutoRightBlue_Cmd(SubSys_DriveTrain driveSubSys, SubSys_PigeonGyro pigeonGyro, SubSys_Hand handSubSys, SubSys_Arm armSubSys) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_hand = handSubSys;
    m_Arm = armSubSys;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new Cmd_whatever the arm one is
        //new InstantCommand(m_hand::CloseHand, m_hand),
        // new Cmd_HighLevelRotate(armSubSys),
        // new Cmd_HighLevelExtend(armSubSys),
        new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "alliancered", true, true));
  }
}
