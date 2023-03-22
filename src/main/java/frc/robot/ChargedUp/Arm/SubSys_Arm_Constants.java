// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import frc.robot.Constants.Robot;

/** Add your docs here. */
public class SubSys_Arm_Constants {
  public static final double ArmShoulderZeroAngle = 44.7; // Degrees
  public static final double ArmExtensionEncoderFullyExtendedDegrees = 3460; // Degrees
  public static final double ArmExtensionDegToMetersFactor =
      (Robot.Dimensions.Arm.ArmMaxExtensionLength - Robot.Dimensions.Arm.ArmMinLength)
          / ArmExtensionEncoderFullyExtendedDegrees;

  public static final double ArmShoulderMinAngle = -215.0; // Deg
  public static final double ArmShoulderMaxAngle = 40.0; // Deg

  public class ArmShoulder {
    public static final boolean ForwardSoftLimitEnable = false;
    public static final double ForwardSoftLimitThreshold = ArmShoulderMaxAngle;
    public static final boolean ReverseSoftLimitEnable = false;
    public static final double ReverseSoftLimitThreshold = ArmShoulderMinAngle;

    public static final double openloopRamp = 0.5; // Seconds to Max Speed

    public class FF {
      public static final double kS = 0;
      public static final double kV = 0;
    }

    public class PID {
      public static final double kP = 2.5;
      public static final double kI = 0.01;
      public static final double kD = 0.0;
      public static final double IntegralZone = 20; // Sensor Units
      public static final double closedLoopRamp = 0.5; // Seconds to Max Speed
      public static final double allowableClosedLoopError = 1.0; // Sensor Units
      public static final double atSetpointAllowableError = 15; // Sensor Units
    }
  }

  public class ArmExtension {
    public static final boolean ForwardSoftLimitEnable = false;
    public static final double ForwardSoftLimitThreshold = ArmExtensionEncoderFullyExtendedDegrees;
    public static final boolean ReverseSoftLimitEnable = false;
    public static final double ReverseSoftLimitThreshold = 0;

    public static final double openloopRamp = 0.5; // Seconds to Max Speed

    public class FF {
      public static final double kS = 0;
      public static final double kV = 0;
    }

    public class PID {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double closedLoopRamp = 0.5; // Seconds to Max Speed
      public static final double allowableClosedLoopError = 1.0; // Degrees
    }
  }
}
