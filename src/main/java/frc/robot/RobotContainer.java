/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.AutoCommands.Auto_ChargeBlue_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_BlueLeave_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_MiddleChargeBlue_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_ChargeRed_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_RedLeave_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_MiddleChargeRed_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_OneConeBlue_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_OneConeRed_Cmd;
import frc.robot.ChargedUp.ColorSensor.SubSys_ColorSensor;
// import frc.robot.ChargedUp.DistanceSensor.SubSys_DistanceSensor;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
// import frc.robot.ChargedUp.Hand.Cmd.Cmd_HandWithSensor;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.MecanumDrive.SubSys_MecanumDrive;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmds_PathPlanner.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /** **** Library Components */

  // ---- Power Distribution
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // ---- NavXGyro
  // public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  // ---- Pigeon2
  public final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();

  // ---- LimeLight
  private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // ---- Drive Subsystem (Swerve)
  public final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);
  // private final PDPSubSys m_PDPSubSys = new PDPSubSys();

  // public final NavXGyroSubSys m_NavXGyroSubSys = new NavXGyroSubSys();

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();

  // public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  // ---- Hand
  public final SubSys_Hand handSubSys = new SubSys_Hand();

  /*
   ***** Charged Up Componentes
   */

  // ---- Driver Station
  public final SubSys_DriverStation driverStation = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /*
   ***** Auto Commands
   */
  /*

  private final Command m_Auto_PathPlanner_Test_Cmd =
      new DriveSubSys_PathPlanner_Test_Cmd(driveSubSys);

  private final Command m_Auto_PP_FollowTraj_Cmd =
      new DriveSubSys_PP_FollowTraj_Cmd("New New Path",driveSubSys);

  private final Command ihopethisworks =
      new DriveSubSys_PathPlanner_Test_Cmd(driveSubSys);
  */

  private final Command m_chargeBlue =
  new Auto_ChargeBlue_Cmd(driveSubSys, gyroSubSys);

  private final Command m_chargeRed =
  new Auto_ChargeRed_Cmd(driveSubSys, gyroSubSys);

  private final Command m_blueleave =
  new Auto_BlueLeave_Cmd(driveSubSys, gyroSubSys);

  private final Command m_redleave =
  new Auto_RedLeave_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middlechargeBlue =
  new Auto_MiddleChargeBlue_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middlechargeRed =
  new Auto_MiddleChargeRed_Cmd(driveSubSys, gyroSubSys);

  private final Command m_OneConeBlue =
  new Auto_OneConeBlue_Cmd(driveSubSys, gyroSubSys);

  private final Command m_OneConeRed =
  new Auto_OneConeRed_Cmd(driveSubSys, gyroSubSys);
  /*
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    /** ***** Control System Components */
    // handSubSys.setDefaultCommand(
    //     new Cmd_HandWithSensor(
    //         handSubSys, colorSubSys, distanceSubsys, () -> driverStation.HandSensorBtn()));
    // ---- Drive Subsystem Default Command
    driveSubSys.setDefaultCommand(
        new Cmd_SubSys_DriveTrain_JoysticDefault(
            driveSubSys,
            () -> driverStation.DriveFwdAxis(),
            () -> driverStation.DriveStrAxis(),
            () -> driverStation.DriveRotAxis(),
            true,
            () -> driverStation.RotateLeftPt(),
            () -> driverStation.RotateRightPt()));

    // Sendable Chooser
    // m_chooser.setDefaultOption("Auto_BasicRevHighGoalRev_Cmd", m_Auto_BasicRevHighGoalRev_Cmd);
    // m_chooser.addOption("Auto_BasicRevLowGoalRev", m_Auto_BasicRevLowGoalRev_Cmd);
    // m_chooser.addOption("Auto_AS_RevHighGoalRev_Cmd", m_Auto_AS_RevHighGoalRev_Cmd);
    // m_chooser.addOption("Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd",
    // m_Auto_AS_T13CnrtoT13HighGoaltoB2toShoot_Cmd);
    // m_chooser.addOption("Auto_PathPlanner_Test_Cmd", m_Auto_PathPlanner_Test_Cmd);
    // m_chooser.addOption("Auto_PP_FollowTraj_Cmd", m_Auto_PP_FollowTraj_Cmd);
    // m_chooser.addOption("goodluck", ihopethisworks);
    // m_chooser.setDefaultOption("Drive4Distance", m_Drive4Distance_Cmd);
    // m_chooser.addOption("BasicAutoLowWait", m_BasicAutoLowWaitCmd);
    // m_chooser.addOption("BasicAutoHigh", m_BasicAutoHighCmd);
    // m_chooser.addOption("BasicAutoHighExtraBalls", m_BasicAutoHighExtraBallsCmd);
    // m_chooser.addOption("HighshotAuto", m_LeftCenterHigh_Cmd);
    m_chooser.setDefaultOption("chargeblue", m_chargeBlue);
    m_chooser.addOption("leaveblue", m_blueleave);
    m_chooser.addOption("middlechargeblue", m_middlechargeBlue);
    m_chooser.addOption("chargered", m_chargeRed);
    m_chooser.addOption("redleave", m_redleave);
    m_chooser.addOption("middlechargered", m_middlechargeRed);
    m_chooser.addOption("oneconeblue", m_OneConeBlue);
    m_chooser.addOption("oneconered", m_OneConeRed);
   
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gyro Reset Command Button
    driverStation.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStation.CloseHandButton.onTrue(new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStation.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Gyro Reset Command Button
    driverStation.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    driverStation.TestButton.whileTrue(
        // new Cmd_SubSys_DriveTrain_Rotate2Heading(driveSubSys, 90)

        /*
        new Cmd_SubSys_DriveTrain_Drive4Distance(
          driveSubSys,
          .5,
          .5,
          0)
        */

        new Auto_ChargeBlue_Cmd(driveSubSys, gyroSubSys));
  }

  // when test button is pressed run the rotate to heading command to a random number between 0 and
  // 360

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // m_DriveSubSys.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    // return m_BasicAutoCmd;
    return m_chooser.getSelected();
  }
}
