// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubSys_Arm extends SubsystemBase {

  private TalonFX Arm_ShoulderMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderMtr_CAN_ID);
  private TalonFX Arm_ShoulderFollowerMotor = new TalonFX(Constants.CAN_IDs.ArmShoulderFollowerMtr_CAN_ID);

  private CANCoder Arm_ShoulderCanCoder = new CANCoder(Constants.CAN_IDs.ArmShoulderCANCoder_CAN_ID);

  private TalonFX Arm_ExtensionMotor = new TalonFX(Constants.CAN_IDs.ArmExtensionMtr_CAN_ID);

  private CANCoder Arm_ExtensionCanCoder = new CANCoder(Constants.CAN_IDs.ArmExtensionCANCoder_CAN_ID);
  
  // STOP Switch
  private DigitalInput stopSwitch = new DigitalInput(0);
  private Boolean isSlowSwitchClosed;
  // SLOW Switch
  private DigitalInput slowSwitch = new DigitalInput(1);
  private Boolean isStopSwitchClosed;

  public SubSys_Arm() {
    //* PID Primary */

    // Shoulder
    Arm_ShoulderMotor.config_kP(Const_Arm.HardwareConfigs.PID_PRIMARY, 0.05);

    // Extension
    Arm_ExtensionMotor.config_kP(Const_Arm.HardwareConfigs.PID_PRIMARY, 0.05);

    //* PID Secondary */

    // Shoulder
    Arm_ShoulderMotor.config_kP(Const_Arm.HardwareConfigs.PID_SECONDARY, 0.05);

    // Extension
    Arm_ExtensionMotor.config_kP(Const_Arm.HardwareConfigs.PID_SECONDARY, 0.05);

    //* Arm Shoulder CanCoder */
    Arm_ShoulderCanCoder.configFactoryDefault();
    Arm_ShoulderCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ShoulderCanCoder.configMagnetOffset(Const_Arm.kOffsetTo0, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ShoulderCanCoder.setPositionToAbsolute();

    //* Arm Shoulder Motor */
    Arm_ShoulderMotor.configFactoryDefault(); // Reset to factory defaults
    Arm_ShoulderMotor.setInverted(false);
    Arm_ShoulderMotor.setNeutralMode(NeutralMode.Brake); // Set to brake mode
    Arm_ShoulderMotor.configForwardSoftLimitEnable(true); // Enable soft limits
    Arm_ShoulderMotor.configReverseSoftLimitEnable(true); // Enable soft limits
    Arm_ShoulderMotor.configForwardSoftLimitThreshold(Const_Arm.kShoulderForwardSoftLimit, Const_Arm.HardwareConfigs.TIMEOUT_MS); // Set soft limits
    Arm_ShoulderMotor.configReverseSoftLimitThreshold(Const_Arm.kShoulderReverseSoftLimit, Const_Arm.HardwareConfigs.TIMEOUT_MS); // Set soft limits


    // Integrated
    Arm_ShoulderMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ShoulderMotor.configIntegratedSensorOffset(0, Const_Arm.HardwareConfigs.TIMEOUT_MS);

    // Remote
    Arm_ShoulderMotor.configRemoteFeedbackFilter(Arm_ShoulderCanCoder, Const_Arm.HardwareConfigs.REMOTE_0, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Const_Arm.HardwareConfigs.PID_PRIMARY, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ShoulderMotor.configSelectedFeedbackCoefficient(360 / 4096, Const_Arm.HardwareConfigs.PID_PRIMARY, Const_Arm.HardwareConfigs.TIMEOUT_MS);

    //* Arm Shoulder Follower Motor */
    Arm_ShoulderFollowerMotor.configFactoryDefault();
    Arm_ShoulderFollowerMotor.setInverted(true);
    Arm_ShoulderFollowerMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ShoulderFollowerMotor.follow(Arm_ShoulderMotor); //Use same output as shoulder motor 


    //* Arm Extension CanCoder */
    Arm_ExtensionCanCoder.configFactoryDefault();
    Arm_ExtensionCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Const_Arm.HardwareConfigs.TIMEOUT_MS);

    //* Arm Extension Motor */
    Arm_ExtensionMotor.configFactoryDefault();
    Arm_ExtensionMotor.setInverted(false);
    Arm_ExtensionMotor.setNeutralMode(NeutralMode.Brake);

    // Integrated
    Arm_ExtensionMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ExtensionMotor.configIntegratedSensorOffset(0, Const_Arm.HardwareConfigs.TIMEOUT_MS);

    // Remote
    Arm_ExtensionMotor.configRemoteFeedbackFilter(Arm_ExtensionCanCoder, Const_Arm.HardwareConfigs.REMOTE_0, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Const_Arm.HardwareConfigs.PID_PRIMARY, Const_Arm.HardwareConfigs.TIMEOUT_MS);
    Arm_ExtensionMotor.configSelectedFeedbackCoefficient(7.854 / 4096, Const_Arm.HardwareConfigs.PID_PRIMARY, Const_Arm.HardwareConfigs.TIMEOUT_MS);
  }

  // *Math methods
  // z = height

  public double getCameraHeight() {
    return Const_Arm.kARM_SHOULDER_z - (Math.cos(getShoulderRotationRadians()) * getArmExtension());
  }
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
      case 0: // Unlocked
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        // SmartDashboard.putString("RotateArmLockValue", "Specified");
        break;
      case 1: // Pos-No-Neg max
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
        // SmartDashboard.putString("RotateArmLockValue", "Specified");
        break;
      case 2: // Neg-No-Pos min
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
      case 0: // Unlocked
        // SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        break;
      case 1: // Extend No Retract
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
        break;
      case 2: // Retract No Extend
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, PercentOutput));
        break;
      case 3: // Slow
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        if (PercentOutput > 0) { // Extending is not slowed
          Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        }
        if (PercentOutput > 0) { // Retracting is slowed
          Arm_ExtensionMotor.set(
              TalonFXControlMode.PercentOutput, PercentOutput * Const_Arm.kSLOW_MULTIPLIER);
        }
        break;
      default:
        //  SmartDashboard.putString("ExtendArmLockValue", "Not Specified");
        break;
    }
  }
  // *Motor methods (-InBoundsMethods-)
  public void RotateArm_InBounds(double PercentOutput) {
    double ArmShoulderAngle = Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);

    if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "No boundary hit");
      Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_z) {
      if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90) RotateArm(2, PercentOutput);
      if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180) RotateArm(1, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "Width boundary hit");
      if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90) RotateArm(1, PercentOutput);
      if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180) RotateArm(2, PercentOutput);
      if (ArmShoulderAngle >= 180 && ArmShoulderAngle < 270) RotateArm(1, PercentOutput);
      if (ArmShoulderAngle >= 270 && ArmShoulderAngle < 360) RotateArm(2, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_z && currentLength > Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "Both boundary hit");
      RotateArm(0, 0);
    }
  }

  public void rotateArmMinMax(double percentOutput, double min, double max) {
    double ArmShoulderAngle = Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    if (ArmShoulderAngle > max && ArmShoulderAngle < max + 30) {
      RotateArm(1, percentOutput);
    } else if (ArmShoulderAngle < min && ArmShoulderAngle > min - 30) {
      RotateArm(2, percentOutput);
    } else {
      RotateArm(0, percentOutput);
    }
  }

  public void ExtendArm_InBounds(double PercentOutput) {
    double ArmShoulderAngle = Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);

    // if (isSlowSwitchClosed || inSlowArea && !isSwitch1Closed) {
    //   ExtendArm(3, PercentOutput);

    //   if (Arm_ExtensionCanCoder.getPosition() > Const_Arm.kSWITCH_ARM_LOCATION) {inSlowArea =
    // false; }
    //   else { inSlowArea = true;}

    //   if (isSlowSwitchClosed) {
    //     Arm_ExtensionCanCoder.setPosition(Const_Arm.kSWITCH_ARM_LOCATION);
    //   }
    // }
    if (isStopSwitchClosed) {
      ExtendArm(1, PercentOutput);
      Arm_ExtensionCanCoder.setPosition(0);
    }
    if (!isStopSwitchClosed && !isSlowSwitchClosed) {
      if (currentHeight < Const_Arm.kMAX_EXTENSION_z
          && currentLength < Const_Arm.kMAX_EXTENSION_x) {
        ExtendArm(0, PercentOutput);
      } else {
        ExtendArm(2, PercentOutput);
      }
    }
  }
  
  /**
   * Arm extension control method that limits the extension of the arm within the specified minimum
   * and maximum values.
   *
   * @param PercentOutput the percentage of output/speed to use for the arm extension motor
   * @param min the minimum allowed extension value for the arm
   * @param max the maximum allowed extension value for the arm
   */
  public void armExtentionMinMax(double PercentOutput, double min, double max) {
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    if (isStopSwitchClosed) {
      ExtendArm(2, PercentOutput);
      Arm_ExtensionCanCoder.setPosition(0);
    }
    if (!isStopSwitchClosed) {
      if (ArmExtendLength < max) {
        ExtendArm(0, PercentOutput);
      }
      if (ArmExtendLength > max) { 
        ExtendArm(1, PercentOutput);
      }
    }
  }

  public double getShoulderRotation() {
    return Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
  }
  public double getShoulderRotationRadians() {
    double degrees = Arm_ShoulderCanCoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    return Math.toRadians(degrees);
  }
  public double getArmExtension() {
    return Arm_ExtensionMotor.getSelectedSensorPosition();
  } 
  public double resetExtendCanCoder() {
    return Arm_ExtensionMotor.getSelectedSensorPosition();
  }


  //! --- ON MOTOR PIDs --- \\


  public void armRotationMoveToPos(double setPositionDegrees){
    // First, select config slot 0 for the TalonFX
    Arm_ShoulderMotor.selectProfileSlot(Const_Arm.HardwareConfigs.SLOT_0, Const_Arm.HardwareConfigs.PID_PRIMARY);

    // Then calculate the position in encoder ticks and drive the motor to that position
     setPositionDegrees *= (4096 / 360);
    Arm_ShoulderMotor.set(TalonFXControlMode.Position, setPositionDegrees);
  }

  public void armExtensionMoveToPos(double setPositionCM){
    // First, select config slot 0 for the TalonFX
    Arm_ExtensionMotor.selectProfileSlot(Const_Arm.HardwareConfigs.SLOT_0, Const_Arm.HardwareConfigs.PID_PRIMARY);

    // Then calculate the position in encoder ticks and drive the motor to that position
    setPositionCM *= (4096 / 7.854) ;
    Arm_ExtensionMotor.set(TalonFXControlMode.Position, setPositionCM);
  }

  public Boolean armRotationAtCorrectPosition() {
        //Check if we're close enough
        if (Arm_ShoulderMotor.getClosedLoopError() < +Const_Arm.kErrThreshold &&
        Arm_ShoulderMotor.getClosedLoopError() > -Const_Arm.kErrThreshold) {
        
        ++Const_Arm._withinThresholdLoops;
        } else {
         Const_Arm._withinThresholdLoops = 0;
        }
        return (Const_Arm._withinThresholdLoops > Const_Arm.kLoopsToSettle);
  }

  public Boolean armExtensionAtCorrectPosition() {
        //Check if we're close enough
        if (Arm_ExtensionMotor.getClosedLoopError() < +Const_Arm.kErrThreshold &&
        Arm_ExtensionMotor.getClosedLoopError() > -Const_Arm.kErrThreshold) {
        
        ++Const_Arm._withinThresholdLoops;
        } else {
         Const_Arm._withinThresholdLoops = 0;
        }
        return (Const_Arm._withinThresholdLoops > Const_Arm.kLoopsToSettle);
  }

  @Override
  public void periodic() {
 
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SubSys_Arm_ShoulderCanCoder_AbdPosition",
      Arm_ShoulderCanCoder.getAbsolutePosition());

    SmartDashboard.putNumber("SubSys_Arm_ShoulderMotor_Position", 
      Arm_ShoulderMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("SubSys_Arm_ExtendCanCoder_Position", 
      Arm_ExtensionCanCoder.getPosition());
      
    SmartDashboard.putNumber("SubSys_Arm_ExtendMotor_Position", 
      Arm_ExtensionMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("SubSys_Arm_ShoulderCanCoder_Position", 
      Arm_ShoulderCanCoder.getPosition());

    SmartDashboard.putNumber("RobotHeight",
      getHeightOfArmFromBase(
        Arm_ShoulderMotor.getSelectedSensorPosition(),
        Arm_ExtensionMotor.getSelectedSensorPosition()));

    SmartDashboard.putNumber("RobotWidth",
      getLengthOfArmFromBase(
        Arm_ShoulderMotor.getSelectedSensorPosition(),
        Arm_ExtensionMotor.getSelectedSensorPosition()));

    isSlowSwitchClosed = !slowSwitch.get();

    isStopSwitchClosed = !stopSwitch.get();

    SmartDashboard.putBoolean("isSwitchClosed", isStopSwitchClosed);
  }
}
