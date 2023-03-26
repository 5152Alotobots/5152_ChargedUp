// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AutoCommands.Basic;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings.PoseEstimationStrategy;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

/**
 * *Link For PathPlaner *
 * https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e65ac68f1d_0_53
 */
public class Auto_rightredcharge_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Bling subSys_Bling;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_rightredcharge_Cmd(SubSys_DriveTrain driveSubSys, SubSys_PigeonGyro pigeonGyro, SubSys_Bling subSys_Bling) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    this.subSys_Bling = subSys_Bling;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new Cmd_whatever the arm one is
        new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
            driveSubSys, "rightredcharge", true, true, Alliance.Red, PoseEstimationStrategy.OdometryONLY),
        new Cmd_AutoBalance(pigeonGyro, driveSubSys, subSys_Bling));
  }
}
