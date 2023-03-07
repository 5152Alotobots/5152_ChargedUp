// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class SubSys_Arm extends SubsystemBase {

  private CANCoderConfiguration armExtensionCanCoderConfiguration = new CANCoderConfiguration();

  private TalonFX Arm_ShoulderMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderMtr_CAN_ID);
  private TalonFX Arm_ShoulderFollowerMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderFollowerMtr_CAN_ID);

  private TalonFXConfiguration arm_ShoulderMotorConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration arm_ShoulderFollowerMotorConfiguration = new TalonFXConfiguration();

  private CANCoder Arm_ShoulderCanCoder =new CANCoder(Constants.CAN_IDs.ArmShoulderCANCoder_CAN_ID);

  private TalonFX Arm_ExtensionMotor = new TalonFX(Constants.CAN_IDs.ArmExtensionMtr_CAN_ID);

  private CANCoder Arm_ExtensionCanCoder = new CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID);

  private TalonFXConfiguration ArmExtensionConfig = new TalonFXConfiguration();

  //STOP Switch
  private DigitalInput limitSwitch1 = new DigitalInput(0);
  private Boolean isSwitch2Closed;
  //SLOW Switch
  private DigitalInput limitSwitch2 = new DigitalInput(1);
  private Boolean isSwitch1Closed;
  private Boolean inSlowArea;

  // private CANCoder armExtensionCanCoder = new
  // CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID); NOT USED

  public SubSys_Arm() {
    // *motor configs */
    Arm_ShoulderMotor.configFactoryDefault();
    Arm_ShoulderMotor.setInverted(false);
    Arm_ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ShoulderMotor.configRemoteFeedbackFilter(Arm_ShoulderCanCoder, 0);
    Arm_ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    Arm_ShoulderMotor.configSelectedFeedbackCoefficient(360 / 4096);

    Arm_ShoulderFollowerMotor.configFactoryDefault();
    Arm_ShoulderFollowerMotor.setInverted(true);
    Arm_ShoulderFollowerMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ShoulderFollowerMotor.follow(Arm_ShoulderMotor);

    Arm_ShoulderCanCoder.configFactoryDefault();

    Arm_ExtensionMotor.configFactoryDefault();
    Arm_ExtensionMotor.setInverted(true);
    Arm_ExtensionMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ExtensionMotor.configRemoteFeedbackFilter(Arm_ExtensionCanCoder, 0);
    Arm_ExtensionMotor.configRemoteFeedbackFilter(Arm_ShoulderCanCoder, 0);
    Arm_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    Arm_ExtensionMotor.configSelectedFeedbackCoefficient(1 / 4096);
  }

  // *Math methods
  // z = height
  public double getHeightOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    if (ArmShoulderAngle >= 0 && ArmShoulderAngle <= 180) {
      double radians = Math.toRadians(ArmShoulderAngle); // Convert from degrees to radians
      return Const_Arm.kARM_SHOULDER_z
          + (Math.sin(radians)
              * (ArmExtensionLength + Const_Arm.kHAND_LENGTH)); // if angle is between 0-180
    } else
      return Const_Arm
          .kARM_SHOULDER_z; // if not above the pivot point then hight is the pivot point
  }
  // x = offset
  public double getLengthOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    double radians = Math.toRadians(ArmShoulderAngle); // Convert from degrees to radians
    return (Const_Arm.kARM_SHOULDER_x
            + Math.cos(radians) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH))
        - Const_Arm
            .kROBOT_WIDTH; // no if needed because we simply subtract the robots width and it does
    // not matter if this becomes negative
  }

  // *Motor methods (-Basic-)
    /**
     * @param lockValue 0 = Unlocked, 1 = Pos-No-Neg 2, = Neg-No-Pos
     * @param PercentOutput -1 - 1 double
     */
    public void RotateArm(int lockValue, double PercentOutput) {
      switch (lockValue) {
        case 0: //Unlocked
          Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
          // SmartDashboard.putString("RotateArmLockValue", "Specified");
         break;
        case 1: //Pos-No-Neg max
          Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
          // SmartDashboard.putString("RotateArmLockValue", "Specified");
         break;
        case 2: //Neg-No-Pos min
          Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, PercentOutput));
          // SmartDashboard.putString("RotateArmLockValue", "Specified");
         break;
        default:
          // SmartDashboard.putString("RotateArmLockValue", "Not Specified");
         break;
      }
    }
  //   /**
  //    * @param lockValue 0 = Unlocked, 1 = Ext-No-Rtc, 2 = Rtc-No-Ext, 3 = Slow
  //    * @param PercentOutput -1 - 1 double
  //    */
    public void ExtendArm(int lockValue, double PercentOutput) {
      switch (lockValue) {
        case 0: //Unlocked
        // SmartDashboard.putString("ExtendArmLockValue", "Specified");
          Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
         break;
        case 1: //Extend No Retract 
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
          Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
         break;
        case 2: //Retract No Extend
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
          Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, PercentOutput));
         break;
        case 3: //Slow
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
          if (PercentOutput > 0) { //Extending is not slowed
            Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
          } 
          if (PercentOutput > 0) { //Retracting is slowed
            Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput * Const_Arm.kSLOW_MULTIPLIER);
          }
         break; 
        default:
        //  SmartDashboard.putString("ExtendArmLockValue", "Not Specified");
          break;
      }
    }
  // *Motor methods (-InBoundsMethods-)
    public void RotateArm_InBounds(double PercentOutput) {
        double ArmShoulderAngle = Arm_ShoulderCanCoder.getAbsolutePosition()-Const_Arm.kOffsetTo0;
        double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();
    
        double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
        double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    
        if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
          SmartDashboard.putString("Boundary", "No boundary hit");
          Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        }
    
        if (currentHeight > Const_Arm.kMAX_EXTENSION_z) {
          if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90)
            RotateArm(2, PercentOutput);
          if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180)
            RotateArm(1, PercentOutput);
        }
    
        if (currentHeight > Const_Arm.kMAX_EXTENSION_x) {
          SmartDashboard.putString("Boundary", "Width boundary hit");
          if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90)
            RotateArm(1, PercentOutput);
          if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180)
            RotateArm(2, PercentOutput);
          if (ArmShoulderAngle >= 180 && ArmShoulderAngle < 270)
            RotateArm(1, PercentOutput);
          if (ArmShoulderAngle >= 270 && ArmShoulderAngle < 360)
            RotateArm(2, PercentOutput);
        }
    
        if (currentHeight > Const_Arm.kMAX_EXTENSION_z && currentLength > Const_Arm.kMAX_EXTENSION_x) {
          SmartDashboard.putString("Boundary", "Both boundary hit");
          RotateArm(0, 0);
        }
    }
    public void ExtendArm_InBounds(double PercentOutput) {
      double ArmShoulderAngle = Arm_ShoulderCanCoder.getAbsolutePosition()-Const_Arm.kOffsetTo0;
      double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();
  
      double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
      double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);

      // if (isSwitch2Closed || inSlowArea && !isSwitch1Closed) {
      //   ExtendArm(3, PercentOutput);

      //   if (Arm_ExtensionCanCoder.getPosition() > Const_Arm.kSWITCH_ARM_LOCATION) {inSlowArea = false; }
      //   else { inSlowArea = true;}

      //   if (isSwitch2Closed) {
      //     Arm_ExtensionCanCoder.setPosition(Const_Arm.kSWITCH_ARM_LOCATION);
      //   }
      // } 
      if (isSwitch1Closed) {
        ExtendArm(1, PercentOutput);
        Arm_ExtensionCanCoder.setPosition(0);
      }
      if (!isSwitch1Closed && !isSwitch2Closed) {
        if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
          ExtendArm(0, PercentOutput);
        }
        else {
          ExtendArm(2, PercentOutput);
        }
      }
    }


  


  public double getShoulderRotation() {
    return Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
  }
  public void resetExtendCanCoder() {
    Arm_ExtensionCanCoder.setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "SubSys_Arm_ShoulderCanCoder_CalculatedPOS", getShoulderRotation());
        SmartDashboard.putNumber(
          "SubSys_Arm_ShoulderCanCoder_Position", Arm_ShoulderCanCoder.getAbsolutePosition()-Const_Arm.kOffsetTo0);

    SmartDashboard.putNumber(
        "SubSys_Arm_ExtendCanCoder_Position", Arm_ExtensionCanCoder.getPosition());
    SmartDashboard.putNumber(
        "RobotHeight", getHeightOfArmFromBase(Arm_ShoulderMotor.getSelectedSensorPosition(), Arm_ExtensionMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber(
        "RobotWidth", getLengthOfArmFromBase(Arm_ShoulderMotor.getSelectedSensorPosition(), Arm_ExtensionMotor.getSelectedSensorPosition()));
    
    isSwitch2Closed = !limitSwitch2.get();

    isSwitch1Closed = !limitSwitch1.get();

    SmartDashboard.putBoolean(
        "isSwitchClosed", isSwitch2Closed);
  }
}

