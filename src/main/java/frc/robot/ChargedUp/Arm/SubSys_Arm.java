// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubSys_Arm extends SubsystemBase {
  /** Creates a new SubSys_Arm. */
  private TalonFX ArmShoulderMotor;

  private CANCoder ArmShoulderEncoder;
  private TalonFX ArmExtensionMotor;
  private CANCoder ArmExtensionEncoder;

  public SubSys_Arm() {

    /* ArmShoulderMotor Config */
    this.ArmShoulderMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderMtr_CAN_ID);
    this.ArmShoulderMotor.configFactoryDefault();
    this.ArmShoulderMotor.setInverted(false);
    this.ArmShoulderMotor.setNeutralMode(NeutralMode.Brake);

    /* ArmShoulder Encoder Config */
    this.ArmShoulderEncoder = new CANCoder(Constants.CAN_IDs.ArmShoulderCANCoder_CAN_ID);

    /* ArmExtensionMotor Config */
    this.ArmExtensionMotor = new TalonFX(Constants.CAN_IDs.ArmExtensionMtr_CAN_ID);
    this.ArmExtensionMotor.configFactoryDefault();
    this.ArmExtensionMotor.setInverted(false);
    this.ArmExtensionMotor.setNeutralMode(NeutralMode.Brake);

    /* ArmExtensionEncoder Config */
    this.ArmExtensionEncoder = new CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID);
  }

  /**
   * rotateArmShoulder use this command to rotate the ArmShoulder
   *
   * @param percentCommand double percentCommand (-1 - 1)
   */
  public void rotateArmShoulder(double percentCommand) {
    this.ArmShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
  }
  /**
   * use this command to rotate the ArmShoulder
   *
   * @param angleCommand double angleCommand (0 - 360)
   */
  public void rotateArmShoulder_angleCommand(double angleCommand) {
    this.ArmShoulderMotor.set(TalonFXControlMode.Position, angleCommand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
