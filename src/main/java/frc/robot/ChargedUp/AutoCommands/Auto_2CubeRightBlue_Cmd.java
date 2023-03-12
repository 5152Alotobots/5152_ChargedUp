// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmExtensionPID;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmRotationPID;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelExtend;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelRotate;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.Arm.SubSys_Arm;


public class Auto_2CubeRightBlue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Hand m_hand;
  private final SubSys_Arm m_Arm;
  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_2CubeRightBlue_Cmd(SubSys_DriveTrain driveSubSys, SubSys_PigeonGyro pigeonGyro, SubSys_Hand handSubSys, SubSys_Arm armSubSys) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_hand = handSubSys;
    m_Arm = armSubSys;

    /* Parallel commands */
    ParallelCommandGroup armRotHighLevelAndExtendParallel = new ParallelCommandGroup(
      new Cmd_ArmRotationPID(armSubSys, 145),
      new Cmd_ArmExtensionPID(armSubSys, 42)
    );

    ParallelCommandGroup armRotateAndRetractDriveToPickup1Parallel = new ParallelCommandGroup(
      new Cmd_ArmExtensionPID(armSubSys, 0),
      new Cmd_ArmRotationPID(armSubSys, -43),
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetopickup1", true, true)
    );

    ParallelCommandGroup armRotateAndExtendDriveToDeliver1Parallel = new ParallelCommandGroup(
      new Cmd_ArmRotationPID(armSubSys, 145),
      new Cmd_ArmExtensionPID(armSubSys, 42),
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetodeliver1", false, false)
    );

    ParallelCommandGroup armRotateAndRetractDriveToPickup2Parallel = new ParallelCommandGroup(
      new Cmd_ArmRotationPID(armSubSys, -43),
      new Cmd_ArmExtensionPID(armSubSys, 0),
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetopickup2", true, true)
    );

    ParallelCommandGroup armRotateAndExtendDriveToDeliver2Parallel = new ParallelCommandGroup(
      new Cmd_ArmRotationPID(armSubSys, 145),
      new Cmd_ArmExtensionPID(armSubSys, 42),
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetodeliver2", false, false)
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        armRotHighLevelAndExtendParallel,
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndRetractDriveToPickup1Parallel,
        //USE PHOTONVISION TO FIND CUBE HERE
        new InstantCommand(handSubSys::CloseHand),
        armRotateAndExtendDriveToDeliver1Parallel,
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndRetractDriveToPickup2Parallel,
        //USE PHOTONVISION TO FIND CUBE HERE
        new InstantCommand(handSubSys::CloseHand),
        armRotateAndExtendDriveToDeliver2Parallel
        );
  }
}
