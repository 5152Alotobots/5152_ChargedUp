// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SubSys_DriverStation extends SubsystemBase {
  /** Creates a new DriverStationSubSys. */

  // Driver Controller
  private XboxController m_DriverController = new XboxController(0);

  // Co-Driver Controller
  private XboxController m_CoDriverController = new XboxController(1);

  // AuxDriver Controller
  private XboxController m_AuxDriverController = new XboxController(2);

  public JoystickButton GyroResetButton = new JoystickButton(m_DriverController, 4);
  public JoystickButton CloseHandButton = new JoystickButton(m_DriverController, 6);
  public JoystickButton OpenHandButton = new JoystickButton(m_DriverController, 5);
  public JoystickButton PoseResetButton = new JoystickButton(m_DriverController, 1);
  public JoystickButton TestButton = new JoystickButton(m_DriverController, 3);
  public Trigger TurboButton = new Trigger(() -> m_DriverController.getRawAxis(3) > 0.5);

  public SubSys_DriverStation() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** ***** Library */

  /** ***** Library */

  // ---- Drive Subsystem
  // Drive Fwd Axis
  public double DriveFwdAxis() {
    return -m_DriverController.getRawAxis(1);
  }

  // Drive Strafe Axis
  public double DriveStrAxis() {
    return -m_DriverController.getRawAxis(0);
  }

  // Drive Rotate Axis
  public double DriveRotAxis() {
    return -m_DriverController.getRawAxis(4);
  }

  // Drive RotateLeftPoint
  public boolean RotateLeftPt() {
    return m_DriverController.getRawButton(5);
  }

  // Drive RotateRightPoint
  public boolean RotateRightPt() {
    return m_DriverController.getRawButton(6);
  }

  // ---- Intake
  // ------ IntakeInNOut

  // IntakeInNOut_Intake
  public boolean IntakeInNOut_Intake() {
    return m_CoDriverController.getRawButton(1);
  }

  /*
  // IntakeInOut_ShortShot
  public boolean IntakeInNOut_ShortShot(){
    return m_CoDriverController.getRawButton(2);
  }
  */
  public double GetArmRotateAxis() {
    return m_AuxDriverController.getRawAxis(1);
  }

  public double GetArmExtendAxis() {
    return m_AuxDriverController.getRawAxis(5);
  }
  // IntakeInOut_ShortShot Axis
  public double IntakeInNOut_ShortShotAxis() {
    return m_CoDriverController.getRightTriggerAxis();
  }

  /*
  // IntakeInOut_LongShot
    public boolean IntakeInNOut_LongShot(){
    return m_CoDriverController.getRawButton(4);
  }
  */

  // IntakeInNOut_LongShotAxis
  public double IntakeInNOut_LongShotAxis() {
    return m_CoDriverController.getLeftTriggerAxis();
  }

  // ------IntakeArm

  // IntakeArm_Axis
  public double IntakeArm_Axis() {
    return m_CoDriverController.getRawAxis(1);
  }

  // IntakeLeftTrigger
  public boolean IntakeLeftTrigger() {
    return m_CoDriverController.getRawButton(5);
  }

  // IntakeRightTrigger
  public boolean IntakeRightTrigger() {
    return m_CoDriverController.getRawButton(6);
  }

  // ---- Climber
  // ------ Climber Lift

  // Climber Lift Positive
  public boolean ClimberLiftPositiveBtn() {

    return m_DriverController.getRawButton(4);
  }

  // Climber Lift Negative
  public boolean ClimberLiftNegativeBtn() {
    return m_DriverController.getRawButton(1);
  }

  // ------ Climber Rotator
  public boolean ClimberLiftRotatorPositiveBtn() {
    return m_DriverController.getRawButton(2);
  }

  public boolean ClimberLiftRotatorNegativeBtn() {
    return m_DriverController.getRawButton(3);

    // ---- TEST POSE

  }

  public double HandSensorBtn() {
    boolean buttonValue = m_AuxDriverController.getRawButton(0);
    SmartDashboard.putBoolean("Hand Ready", buttonValue);
    if (buttonValue == true) return 1;
    else return 0;
  }
}
