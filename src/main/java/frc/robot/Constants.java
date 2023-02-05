/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class RobotSettings{
    public static final class DriveTrain{
      // Drive SubSys
    
      // Drive Max Speeds
      public static final double DriveTrainMaxSpd   = 2;                  // m/s
      public static final double DriveTrainMaxAccel = 0.35;                  // m/s^2
      public static final double MaxDriveSubSysRotSpeed = 270*Math.PI/180;    // rad/s
      public static final double MaxDriveSubSysRotAccel = 180*Math.PI/180;   // rad/s^2
    
      // Drive Trajectory Max Speeds
      public static final double kTrajMaxVel = 1.85;    // m/s
      public static final double kTrajMaxAccel = 0.35; // m/s/s
    }

   public static final class Limelight{
      public static final double kCameraHeight = Units.inchesToMeters(35);  // m
      public static final double kCameraAngle = 29.0; // Degrees
    }
  }

  public static final class Field{
    public static final class Hub{
      public static final Translation2d kHubCenter =
        new Translation2d(
          Units.inchesToMeters(324),
          Units.inchesToMeters(162));
      
      public static final Translation2d kH1 = 
        new Translation2d(
          Units.inchesToMeters(308),
          Units.inchesToMeters(130));

      public static final Translation2d kH2 = 
        new Translation2d(
          Units.inchesToMeters(294),
          Units.inchesToMeters(174));

      public static final double kTargetRingHeight = 
        Units.inchesToMeters(105);

      public static final double kTargetRingDist2Ctr =
        Units.inchesToMeters(26);
    }
    
    public static final class Tarmac{
      public static final Translation2d kT11 = 
        new Translation2d(
          Units.inchesToMeters(364),
          Units.inchesToMeters(53));

      public static final Translation2d kT12 = 
        new Translation2d(
          Units.inchesToMeters(281),
          Units.inchesToMeters(51));

      public static final Translation2d kT13 = 
        new Translation2d(
          Units.inchesToMeters(221),
          Units.inchesToMeters(108));
    }
  }
    
  public static final class CAN_IDs {
    public static final int PDP_CAN_ID = 1;
    public static final int PCM_CAN_ID = 2;

    public static final int Pigeon2_ID = 20;
            
    public static final int ArmShoulderMtr_CAN_ID = 30;
    public static final int ArmShoulderCANCoder_CAN_ID = 31;
    public static final int ArmExtensionMtr_CAN_ID = 30;
    public static final int ArmExtensionCANCoder_CAN_ID = 31;

    public static final int FrontLeftDriveMtr_CAN_ID = 51;
    public static final int FrontLeftSteerMtr_CAN_ID = 52;
    public static final int FrontLeftSteerCANCoder_CAN_ID = 53;
    public static final int FrontRightDriveMtr_CAN_ID = 54;
    public static final int FrontRightSteerMtr_CAN_ID = 55;
    public static final int FrontRightSteerCANCoder_CAN_ID = 56;
    public static final int BackLeftDriveMtr_CAN_ID = 57;
    public static final int BackLeftSteerMtr_CAN_ID = 58;       
    public static final int BackLeftSteerCANCoder_CAN_ID = 59; 
    public static final int BackRightDriveMtr_CAN_ID = 60;        
    public static final int BackRightSteerMtr_CAN_ID = 61;
    public static final int BackRightSteerCANCoder_CAN_ID = 62;
  }

  public static final class AnalogInput_IDs{
  }

  public static final class DigitalIO_IDs{
  }

  public static final class PWM_IDs{
  }
}
