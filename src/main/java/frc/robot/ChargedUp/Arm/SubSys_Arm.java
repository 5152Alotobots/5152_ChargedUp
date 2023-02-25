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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubSys_Arm extends SubsystemBase {
  private CANCoderConfiguration armExtensionCanCoderConfiguration = new CANCoderConfiguration();

  private TalonFX Arm_ShoulderMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderMtr_CAN_ID);
  private TalonFX Arm_ShoulderFollowerMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderFollowerMtr_CAN_ID);

  private TalonFXConfiguration arm_ShoulderMotorConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration arm_ShoulderFollowerMotorConfiguration = new TalonFXConfiguration();

  private CANCoder Arm_ShoulderCanCoder = new CANCoder(Constants.CAN_IDs.ArmShoulderCANCoder_CAN_ID);

  private TalonFX ArmExtensionMotor = new TalonFX(Constants.CAN_IDs.ArmExtensionMtr_CAN_ID);
  private TalonFXConfiguration ArmExtensionConfig = new TalonFXConfiguration();
  // private CANCoder armExtensionCanCoder = new CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID); NOT USED
  

  public SubSys_Arm() {
    //*motor configs */
      Arm_ShoulderMotor.configFactoryDefault();
      Arm_ShoulderMotor.setInverted(false);
      Arm_ShoulderMotor.setNeutralMode(NeutralMode.Brake);
      Arm_ShoulderMotor.configRemoteFeedbackFilter(Arm_ShoulderCanCoder, 0);
      Arm_ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
      Arm_ShoulderMotor.configSelectedFeedbackCoefficient(1/4096);
      
      Arm_ShoulderFollowerMotor.configFactoryDefault();
      Arm_ShoulderFollowerMotor.setInverted(true);
      Arm_ShoulderFollowerMotor.setNeutralMode(NeutralMode.Brake);
      Arm_ShoulderFollowerMotor.follow(Arm_ShoulderMotor);

      Arm_ShoulderCanCoder.configFactoryDefault();

      ArmExtensionMotor.configFactoryDefault();
      ArmExtensionMotor.setInverted(false);
      ArmExtensionMotor.setNeutralMode(NeutralMode.Brake);
      ArmExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		  ArmExtensionConfig.remoteFilter0.remoteSensorDeviceID = ((IMotorController) ArmExtensionConfig).getDeviceID(); //Device ID of Remote Source
		  ArmExtensionConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
  }

  //*Math methods
  //z = height
  public double getHeightOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    if (ArmShoulderAngle >= 0 && ArmShoulderAngle <= 180) {
      double radians = Math.toRadians(ArmShoulderAngle); //Convert from degrees to radians 
      return Const_Arm.kARM_SHOULDER_z + (Math.sin(radians) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH)); //if angle is between 0-180
    }
    else 
        return Const_Arm.kARM_SHOULDER_z; //if not above the pivot point then hight is the pivot point 
  }
  //x = offset
  public double getLengthOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    double radians = Math.toRadians(ArmShoulderAngle); //Convert from degrees to radians 
    return (Const_Arm.kARM_SHOULDER_x + Math.cos(radians) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH)) - Const_Arm.kROBOT_WIDTH; //no if needed because we simply subtract the robots width and it does not matter if this becomes negative
  }

  //*Motor methods (-SHOULDER-)

    /** @param angleCommand double angleCommand (0 - 360) */

    public void rotateArmShoulder_angleCommand(double angleCommand){
      Arm_ShoulderMotor.set(TalonFXControlMode.Position, angleCommand);
      //TODO fix this follow cmd 
      Arm_ShoulderFollowerMotor.set(TalonFXControlMode.Position, angleCommand);
    }

    /** @param percentCommand double percentCommand (-1 - 1) */

    public void rotateArmShoulder_feedForward(double percentCommand) {
      Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand, DemandType.ArbitraryFeedForward, 0);
      //Arm_ShoulderFollowerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

   /** @param percentCommand double percentCommand (-1 - 1) */

    public void rotateArmShoulder(double percentCommand) {
      Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
      //Arm_ShoulderFollowerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    /** @param percentCommand double percentCommand (-1 - 1)
     *  @param ArmShoulderAngle double shoulderAngle (0 - 360)
     */

    public void rotate_UntilOuterBoundary(double percentCommand) {
      double ArmShoulderAngle = Arm_ShoulderMotor.getSelectedSensorPosition();
      double ArmExtendLength = ArmExtensionMotor.getSelectedSensorPosition();

      double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
      double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
      
      if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
        SmartDashboard.putString("Boundary", "No boundary hit");
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
      }

      if (currentHeight > Const_Arm.kMAX_EXTENSION_z) {
        if (ArmShoulderAngle >=   0 && ArmShoulderAngle < 90 ) rotateArmShoulder(Math.min(0, percentCommand));
        if (ArmShoulderAngle >=  90 && ArmShoulderAngle < 180) rotateArmShoulder(Math.max(0, percentCommand));
      }

      if (currentHeight > Const_Arm.kMAX_EXTENSION_x) {
        SmartDashboard.putString("Boundary", "Width boundary hit");
        if (ArmShoulderAngle >=   0 && ArmShoulderAngle < 90 ) rotateArmShoulder(Math.max(0, percentCommand));
        if (ArmShoulderAngle >=  90 && ArmShoulderAngle < 180) rotateArmShoulder(Math.min(0, percentCommand));
        if (ArmShoulderAngle >= 180 && ArmShoulderAngle < 270) rotateArmShoulder(Math.max(0, percentCommand));
        if (ArmShoulderAngle >= 270 && ArmShoulderAngle < 360) rotateArmShoulder(Math.min(0, percentCommand));
      }

      if (currentHeight > Const_Arm.kMAX_EXTENSION_z && currentLength > Const_Arm.kMAX_EXTENSION_x) { 
      SmartDashboard.putString("Boundary", "Both boundary hit");
        rotateArmShoulder(0);
      }
    }

  //*Motor methods (-Extend-) 

    /** @param percentCommand double percentCommand (-1 - 1) */

    public void extend(double percentCommand) {
      ArmExtensionMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
    }

    /** @param percentCommand double percentCommand (-1 - 1) */

    public void extend_UntilOuterBoundary(double percentCommand) {
      double ArmShoulderAngle = Arm_ShoulderMotor.getSelectedSensorPosition();
      double ArmExtendLength = ArmExtensionMotor.getSelectedSensorPosition();

      double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
      double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    
      if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
        ArmExtensionMotor.set(TalonFXControlMode.PercentOutput, percentCommand);
      }
      else {
        ArmExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, percentCommand)); //arm can retract but not extend
      }
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SubSys_Arm__ShoulderMotor_Position",Arm_ShoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("SubSys_Arm__ShoulderFollowerMotor_Position",Arm_ShoulderFollowerMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("SubSys_Arm_ShoulderCanCoder_Position",Arm_ShoulderCanCoder.getAbsolutePosition());
  }
}
