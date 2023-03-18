package frc.robot.ChargedUp.Arm;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class Const_Arm {
  // *! Everything is in meters */
  public static final int RotateMotorCanID = 0;
  public static final int ExtendMotorCanID = 0;
  public static final double kARM_SHOULDER_z = .58; // VERTICAL DIRECTION
  public static final double kARM_SHOULDER_x = .05;
  public static final double kHAND_LENGTH = .35;
  public static final double kMAX_EXTENSION_x = 1.20; // .120m or 120cm (width)
  public static final double kMAX_EXTENSION_z = 1.98; // .198m or 198cm (Height)
  public static final double kSWITCH_ARM_LOCATION = 0;
  public static final double kMAX_ROTATION_SPEED = .4;
  public static final double kMAX_EXTENSION_SPEED = .6;
  public static final double kSLOW_EXTENSION_SPEED = .1;
  public static final double kSLOW_MULTIPLIER = .5;
  public static final double kROBOT_WIDTH = .6;
  public static final double kMinAngle = 0;
  public static final double kMaxAngle = 0;
  public static final double kOffsetTo0 = 44.1+90;

  public class ExtendPID {
    public static final double kP = 3.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTOLERANCE = 0.5;
    public static final double kSPEED_TOLERANCE = 0.1;
  }

  public static class Trajectory {

    // Motor specifications
    public static final double MAX_ROT_SPEED_RPM = 142; //with 5:1 and 9:1 gearboxes
    public static final double MAX_MOTOR_TORQUE_Nm = 9.4;

    // System specifications
    public static final double SYSTEM_INERTIA_KG_M_SQUARED = 0.085;
    
    public static final double kShoulderMaxRotSpeed = 
    0.9 * Units.degreesToRadians(MAX_ROT_SPEED_RPM * 2 * Math.PI / 60);
    public static final double kShoulderMaxRotAcelSpeed = 
    0.6 * (MAX_MOTOR_TORQUE_Nm / SYSTEM_INERTIA_KG_M_SQUARED);

  }

  public class Positions {
    public static final double pickup = 0;
  }

  public class HardwareConfigs {
    // Global
    public static final int TIMEOUT_MS = 0;

    // PID slot configs
    public static final int PID_PRIMARY = 0;
    public static final int PID_SECONDARY = 1;

    // Remote sensors configs
    public static final int REMOTE_0 = 0;
    public static final int REMOTE_1 = 1;

  }

  public class PidfPrimary {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kIZone = 0;
    public static final double kAllowableError = 0;
    public static final double kMaxIntegralAccumulator = 0;
    public static final double kPeakOutputForward = 0;
    public static final double kPeakOutputReverse = 0;
    public static final double kNominalOutputForward = 0;
    public static final double kNominalOutputReverse = 0;
    public static final double kNeutralDeadband = 0;
  }

  public class PidfSecondary {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kIZone = 0;
    public static final double kAllowableError = 0;
    public static final double kMaxIntegralAccumulator = 0;
    public static final double kPeakOutputForward = 0;
    public static final double kPeakOutputReverse = 0;
    public static final double kNominalOutputForward = 0;
    public static final double kNominalOutputReverse = 0;
    public static final double kNeutralDeadband = 0;
  }
}
