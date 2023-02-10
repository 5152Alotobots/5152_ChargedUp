// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ChargedUp.Arm.Const_Arm;

public class SubSys_Arm extends SubsystemBase {
  private CANCoderConfiguration armExtensionCanCoderConfiguration = new CANCoderConfiguration();

  private TalonFX ArmShoulderMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderMtr_CAN_ID);
  private CANCoder armShoulderCanCoder = new CANCoder(Constants.CAN_IDs.ArmShoulderCANCoder_CAN_ID);

  private TalonFX ArmExtensionMotor = new TalonFX(Constants.CAN_IDs.ArmExtensionMtr_CAN_ID);
  private CANCoder armExtensionCanCoder = new CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID);

  public SubSys_Arm() {
    //*motor configs */
      ArmShoulderMotor.configFactoryDefault();
      ArmShoulderMotor.setInverted(false);
      ArmShoulderMotor.setNeutralMode(NeutralMode.Brake);

      ArmExtensionMotor.configFactoryDefault();
      ArmExtensionMotor.setInverted(false);
      ArmExtensionMotor.setNeutralMode(NeutralMode.Brake);
     
      armExtensionCanCoderConfiguration.sensorCoefficient = 2 * Math.PI / 4096.0;
      armExtensionCanCoderConfiguration.unitString = "rad";
      armExtensionCanCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
      
      armExtensionCanCoder.configAllSettings(armExtensionCanCoderConfiguration);
  }

  /** rotateArmShoulder
   *    use this command to rotate the ArmShoulder
   * @param percentCommand double percentCommand (-1 - 1)
   */
  public void rotateArmShoulder(double percentCommand) {
    ArmShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
  }
/**
 *     use this command to rotate the ArmShoulder
 * @param angleCommand double angleCommand (0 - 360)
 */
  public void rotateArmShoulder_angleCommand(double angleCommand){
    ArmShoulderMotor.set(TalonFXControlMode.Position, angleCommand);
  }

  public void rotateArmShoulder_feedForward(double percentCommand) {
    ArmShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand, DemandType.ArbitraryFeedForward, 0);
  }
  //z = height
  public double getHeightOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    return Const_Arm.kARM_SHOULDER_z + Math.cos(ArmShoulderAngle) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH);
  }
  //x = offset
  public double getLengthOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    return Const_Arm.kARM_SHOULDER_x + Math.sin(ArmShoulderAngle) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH);
  }
  public void extend(double percentCommand) {
    ArmExtensionMotor.set(TalonFXControlMode.PercentOutput, percentCommand);

  }
  public void extendUntilOuterBoundry(double percentCommand) {

    double currentHeight = getHeightOfArmFromBase(0, 0);
    double currentLength = getLengthOfArmFromBase(0, 0);

    if (currentHeight < Const_Arm.k_MAX_EXTENSION_z && currentLength < Const_Arm.k_MAX_EXTENSION_x) {
      ArmExtensionMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
    }
    else {
      SmartDashboard.putString("Info", "Arm has reached outer boundry");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
