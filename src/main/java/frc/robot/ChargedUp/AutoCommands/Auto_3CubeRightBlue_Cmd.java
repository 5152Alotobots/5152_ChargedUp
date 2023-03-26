// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.ChargeStation.Cmd_AutoBalance;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings.PoseEstimationStrategy;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SwerveDrive.*;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.ChargedUp.PhotonVision.Cmd.Cmd_NavigateToBestVisionTarget;
import frc.robot.ChargedUp.Arm.SubSys_Arm;


public class Auto_3CubeRightBlue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Hand m_hand;
  private final SubSys_Arm m_Arm;
  private final SubSys_Photonvision m_photonvision;
  private final SubSys_Bling m_bling;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_3CubeRightBlue_Cmd(SubSys_DriveTrain driveSubSys, SubSys_PigeonGyro pigeonGyro, SubSys_Hand handSubSys, SubSys_Arm armSubSys, SubSys_Photonvision photonvisionSubSys, SubSys_Bling blingSubSys) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_hand = handSubSys;
    m_Arm = armSubSys;
    m_photonvision = photonvisionSubSys;
    m_bling = blingSubSys;
    

    /* Parallel commands */
    ParallelCommandGroup armRotateAndRetractDriveToPickup1Parallel = new ParallelCommandGroup(
      new Cmd_SubSys_Arm_PosCmd(armSubSys, 42, true, 0.8, true).withTimeout(4), // Lift arm to pickup pos
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetopickup1", true, true, Alliance.Blue, PoseEstimationStrategy.OdometryONLY)
    );

    ParallelCommandGroup armRotateAndExtendDriveToDeliverCone1HighLevelParallel = new ParallelCommandGroup(
      new Cmd_SubSys_Arm_PosCmd(armSubSys, -147.0, true, 1.54, true).withTimeout(4), // Lift arm to high position
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetodeliver1", false, false, Alliance.Blue, PoseEstimationStrategy.OdometryONLY)
    );

    ParallelCommandGroup armRotateAndRetractDriveToPickup2Parallel = new ParallelCommandGroup(
      new Cmd_SubSys_Arm_PosCmd(armSubSys, 42, true, 0.8, true).withTimeout(4), // Lift arm to pickup pos
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetopickup2", true, true, Alliance.Blue, PoseEstimationStrategy.OdometryONLY)
    );

    ParallelCommandGroup armRotateAndExtendDriveToDeliverCone2HighLevelParallel = new ParallelCommandGroup(
      new Cmd_SubSys_Arm_PosCmd(armSubSys, 42, true, 0.8, true).withTimeout(4), // Lift arm to pickup pos
      new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetodeliver2", false, false, Alliance.Blue, PoseEstimationStrategy.OdometryONLY)
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Cmd_SubSys_Arm_PosCmd(armSubSys, -147.0, true, 1.54, true).withTimeout(4), // Lift arm to high position
        new InstantCommand(handSubSys::CloseHand),
        armRotateAndRetractDriveToPickup1Parallel,
        new Cmd_NavigateToBestVisionTarget(driveSubSys, m_photonvision, m_bling, Const_Photonvision.Cameras.frontCamera, Const_Photonvision.Pipelines.Cone),
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndExtendDriveToDeliverCone1HighLevelParallel,
        new InstantCommand(handSubSys::CloseHand),
        armRotateAndRetractDriveToPickup2Parallel,
        new Cmd_NavigateToBestVisionTarget(driveSubSys, m_photonvision, m_bling, Const_Photonvision.Cameras.frontCamera, Const_Photonvision.Pipelines.Cone),
        new InstantCommand(handSubSys::OpenHand),
        armRotateAndExtendDriveToDeliverCone2HighLevelParallel,
        new InstantCommand(handSubSys::CloseHand),
        new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(driveSubSys, "leftbluedrivetocharge", false, false, Alliance.Blue, PoseEstimationStrategy.OdometryONLY) //ONLY IF TIME
        );
  }
}
