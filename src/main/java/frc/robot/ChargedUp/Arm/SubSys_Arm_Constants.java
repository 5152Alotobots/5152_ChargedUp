// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

/** Add your docs here. */
public class SubSys_Arm_Constants {
  public static final double ArmShoulderZeroAngle = 47.1;  // Degrees
    
    // *! Everything is in meters */
  public static final double kMinAngle = 0;
  public static final double kMaxAngle = 0;
  public static final double kOffsetTo0 = 47.1;

  public class FF {
    public static final double kS = 0;
    public static final double kV = 0;
  }

  public class PID {
    public static final double kP = 3.0;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public class Trajectory {
    public static final double kShoulderMaxRotSpeed = 270 * Math.PI / 180; // rad/s
    public static final double kShoulderMaxRotAcelSpeed = 180 * Math.PI / 180; // rad/s^2
  }

  public class Positions {
    public static final double pickup = 0;
  }
}
