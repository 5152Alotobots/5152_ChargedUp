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
import frc.robot.ChargedUp.Arm.Cmd.CmdGrp_MoveToLowPos;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmDefault;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmExtensionPID;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmRotationPID;
import frc.robot.ChargedUp.Arm.Cmd.Cmd_ArmRotationPIDOnboard;
// import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelExtend;
// import frc.robot.ChargedUp.Arm.Cmd.Cmd_HighLevelRotate;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.AutoCommands.Auto_leftbluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftblueescape_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftredcharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightredcharge_Cmd;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.Bling.Cmd.CmdGrp_IdleBlingColorSequence;
import frc.robot.ChargedUp.Bling.Cmd.Cmd_SetBlingColorValue;
import frc.robot.ChargedUp.AutoCommands.Auto_leftredescape_Cmd;
// import frc.robot.ChargedUp.DistanceSensor.SubSys_DistanceSensor;
import frc.robot.ChargedUp.AutoCommands.Auto_middlebluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middleredcharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_proofrightredescape;
import frc.robot.ChargedUp.AutoCommands.Auto_rightbluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightblueescape;
import frc.robot.ChargedUp.AutoCommands.Auto_rightredcharge_Cmd;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.MecanumDrive.Cmd.Cmd_MecanumDriveDefault;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.ChargedUp.PhotonVision.Cmd.Cmd_GetDistanceToTarget;
import frc.robot.ChargedUp.PhotonVision.Cmd.Cmd_NavigateToBestVisionTarget;
import frc.robot.ChargedUp.MecanumDrive.SubSys_MecanumDrive;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;
import frc.robot.ChargedUp.Arm.Cmd.CmdGrp_MoveToLowPos;

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

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  // public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();

  // public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  // ---- Hand
  public final SubSys_Hand handSubSys = new SubSys_Hand();

  // Arm
  public final SubSys_Arm armSubSys = new SubSys_Arm();

  //PhotonVision
  public final SubSys_Photonvision photonvisionSubSys = new SubSys_Photonvision(armSubSys);

  // Bling
    public final SubSys_Bling blingSubSys = new SubSys_Bling();
  /*
   ***** Charged Up Componentes
   */

  // ---- Driver Station
  public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_leftbluecharge = new Auto_leftbluecharge_Cmd(driveSubSys, gyroSubSys, blingSubSys);

  private final Command m_leftblueescape = new Auto_leftblueescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftredcharge = new Auto_leftredcharge_Cmd(driveSubSys, gyroSubSys, blingSubSys);

  private final Command m_leftredescape = new Auto_leftredescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middlebluecharge = new Auto_middlebluecharge_Cmd(driveSubSys, gyroSubSys, blingSubSys);

  private final Command m_middleredcharge = new Auto_middleredcharge_Cmd(driveSubSys, gyroSubSys, blingSubSys);

  private final Command m_rightbluecharge = new Auto_rightbluecharge_Cmd(driveSubSys, gyroSubSys, blingSubSys);

  private final Command m_rightblueescape = new Auto_rightblueescape(driveSubSys, gyroSubSys, blingSubSys);

//   private final Command m_OneConeRed = new Auto_OneConeRed_Cmd(driveSubSys, gyroSubSys);

  private final Command m_proofrightredescape =
      new Auto_proofrightredescape(driveSubSys, gyroSubSys, blingSubSys);
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    /** ***** Control System Components */
    armSubSys.setDefaultCommand(
        new Cmd_ArmDefault(
            armSubSys,
            () -> driverStationSubSys.GetArmRotateAxis(),
            () -> driverStationSubSys.GetArmExtendAxis()));

    // handSubSys.setDefaultCommand(new Cmd_HandWithSensor(
    //  handSubSys,
    //  colorSubSys,
    //  distanceSubsys,
    //  () ->  driverStationSubSys.HandSensorBtn())
    // );

    // mecanumDriveSubSys.setDefaultCommand(
    //    new Cmd_MecanumDriveDefault(
    //        mecanumDriveSubSys,
    //        () -> driverStationSubSys.DriveFwdAxis(),
    //        () -> driverStationSubSys.DriveStrAxis(),
    //        () -> driverStationSubSys.DriveRotAxis()));

    driveSubSys.setDefaultCommand(
        new Cmd_SubSys_DriveTrain_JoysticDefault(
            driveSubSys,
            () -> driverStationSubSys.DriveFwdAxis(),
            () -> driverStationSubSys.DriveStrAxis(),
            () -> driverStationSubSys.DriveRotAxis(),
            true,
            () -> driverStationSubSys.RotateLeftPt(),
            () -> driverStationSubSys.RotateRightPt(),
            () -> driverStationSubSys.DrivePerfModeAActive(),
            () -> driverStationSubSys.DrivePerfModeBActive()));

        photonvisionSubSys.setDefaultCommand(
            new Cmd_GetDistanceToTarget(photonvisionSubSys, handSubSys, Const_Photonvision.Cameras.frontCamera, 1)
        );
    blingSubSys.setDefaultCommand(
        new CmdGrp_IdleBlingColorSequence(
            blingSubSys,
            Const_Bling.Controllers.controller1));

    // Sendable Chooser

    // Sendable Chooser
    m_chooser.addOption("leftbluecharge", m_leftbluecharge);
    m_chooser.addOption("leftblueescape", m_leftblueescape);
    m_chooser.addOption("leftredcharge", m_leftredcharge);
    m_chooser.addOption("leftredescape", m_leftredescape);
    m_chooser.addOption("middlebluecharge", m_middlebluecharge);
    m_chooser.addOption("middleredcharge", m_middleredcharge);
    m_chooser.setDefaultOption("rightbluecharge", m_rightbluecharge);
    m_chooser.addOption("rightblueescape", m_rightblueescape);
    // m_chooser.addOption("rightredcharge", m_rightredcharge);
    m_chooser.addOption("proofrightredescape", m_proofrightredescape);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Use this method to define your
   * button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gyro Reset Command Button
    driverStationSubSys.CloseHandButton.onTrue(
        new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStationSubSys.OpenHandButton.onTrue(
        new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStationSubSys.GyroResetButton.onTrue(
        new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // LEDs
    driverStationSubSys.RequestConeButton.onTrue(new Cmd_SetBlingColorValue(blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Yellow));
    driverStationSubSys.RequestCubeButton.onTrue(new Cmd_SetBlingColorValue(blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Violet));

    // Gyro Reset Command Button
    driverStationSubSys.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    //Test Button
    driverStationSubSys.TestButton.onTrue(
        new Cmd_NavigateToBestVisionTarget(driveSubSys, photonvisionSubSys, blingSubSys, Const_Photonvision.Cameras.frontCamera, Const_Photonvision.Pipelines.Cube)
    );
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
    // m_DriveSubSys.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    // return m_BasicAutoCmd;
    return m_chooser.getSelected();
  }
}
