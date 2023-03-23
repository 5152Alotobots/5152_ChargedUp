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
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.AutoCommands.Auto_leftbluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftblueescape_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftbluecone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftredcharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftredcone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_leftredescape_Cmd;
// import frc.robot.ChargedUp.DistanceSensor.SubSys_DistanceSensor;
import frc.robot.ChargedUp.AutoCommands.Auto_middlebluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middlebluecone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middleblueconeleftescape_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middleredcharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middleredcone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_middleredconeleftescape_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightbluecharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightblueescape_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightbluecone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightredcharge_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightredcone_Cmd;
import frc.robot.ChargedUp.AutoCommands.Auto_rightredescape_Cmd;
import frc.robot.ChargedUp.Bling.Cmd.Cmd_SetBlingColorValue;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
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

  // private final SubSys_LimeLight limeLightSubSys = new SubSys_LimeLight();

  // public final SubSys_MecanumDrive mecanumDriveSubSys = new SubSys_MecanumDrive();

  // public final SubSys_ColorSensor colorSubSys = new SubSys_ColorSensor();

  // public final SubSys_DistanceSensor distanceSubsys = new SubSys_DistanceSensor();
  // ---- Driver Station

  // ---- Hand
  public final SubSys_Hand handSubSys = new SubSys_Hand();

  // Arm
  public final SubSys_Arm armSubSys = new SubSys_Arm(handSubSys.getHandLength());

  public final SubSys_Bling blingSubSys = new SubSys_Bling();
  /*
   ***** Charged Up Componentes
   */

  // ---- Driver Station
  public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();
  // SetUp Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_leftbluecharge = new Auto_leftbluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftblueescape = new Auto_leftblueescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftredcharge = new Auto_leftredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_leftredescape = new Auto_leftredescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middlebluecharge = new Auto_middlebluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_middleredcharge = new Auto_middleredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightbluecharge = new Auto_rightbluecharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightblueescape = new Auto_rightblueescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightredcharge = new Auto_rightredcharge_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightredescape = new Auto_rightredescape_Cmd(driveSubSys, gyroSubSys);

  private final Command m_rightbluecone = new Auto_rightbluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_leftbluecone = new Auto_leftbluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middlebluecone = new Auto_middlebluecone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleblueconeleftescape = new Auto_middleblueconeleftescape_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_rightredcone = new Auto_rightredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_leftredcone = new Auto_leftredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleredcone = new Auto_middleredcone_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);

  private final Command m_middleredconeleftescape = new Auto_middleredconeleftescape_Cmd(driveSubSys, gyroSubSys, armSubSys, handSubSys);
      /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    /** ***** Control System Components */
    armSubSys.setDefaultCommand(
        new Cmd_SubSys_Arm_JoysticDefault(
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

    // Sendable Chooser
    m_chooser.addOption("leftbluecharge", m_leftbluecharge);
    m_chooser.addOption("leftblueescape", m_leftblueescape);
    m_chooser.addOption("leftredcharge", m_leftredcharge);
    m_chooser.addOption("leftredescape", m_leftredescape);
    m_chooser.addOption("middlebluecharge", m_middlebluecharge);
    m_chooser.addOption("middleredcharge", m_middleredcharge);
    m_chooser.setDefaultOption("rightbluecharge", m_rightbluecharge);
    m_chooser.addOption("rightblueescape", m_rightblueescape);
    m_chooser.addOption("rightredcharge", m_rightredcharge);
    m_chooser.addOption("rightredescape", m_rightredescape);
    m_chooser.addOption("leftbluecone", m_leftbluecone);
    m_chooser.addOption("middlebluecone", m_middlebluecone);
    m_chooser.addOption("rightbluecone", m_rightbluecone);
    m_chooser.addOption("middleblueconeleftescape", m_middleblueconeleftescape);
    m_chooser.addOption("leftredcone", m_leftredcone);
    m_chooser.addOption("middleredcone", m_middleredcone);
    m_chooser.addOption("rightredcone", m_rightredcone);
    m_chooser.addOption("middleredconeleftescape", m_middleredconeleftescape);

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
    driverStationSubSys.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStationSubSys.CloseHandButton.onTrue(
        new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

    // Gyro Reset Command Button
    driverStationSubSys.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    driverStationSubSys.GroundPickupButton.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, 45.0, true, 0.8, true));

    driverStationSubSys.HighConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -35.0, true, 1.65, true));

    driverStationSubSys.MidConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_PosCmd(armSubSys, -25.0, true, 1.00, true));

    // CONE/CUBE SIGNALING
    driverStationSubSys.RequestConeButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Yellow));
    driverStationSubSys.RequestCubeButton.onTrue(
        new Cmd_SetBlingColorValue(
            blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Violet));
  }

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
