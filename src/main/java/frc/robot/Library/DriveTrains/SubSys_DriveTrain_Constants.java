// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains;

/** Add your docs here. */
public class SubSys_DriveTrain_Constants {

  // Drive Max Speeds
  public static final double DriveTrainMaxPctOutput = 0.25; // 0-1
  public static final double DriveTrainMaxSpd = 2; // m/s
  public static final double DriveTrainMaxAccel = 0.35; // m/s^2
  public static final double DriveTrainMaxRotPctOutput = 0.25; // 0-1
  public static final double DriveTrainMaxRotSpeed = 270 * Math.PI / 180; // rad/s
  public static final double DriveTrainMaxRotAccel = 180 * Math.PI / 180; // rad/s^2

  public static final class DriveTrainTrajSettings {
    // Drive Trajectory Max Speeds (Trajectories or PID Commands)
    public static final double DriveTrainTrajMaxPctOutput = 1; // 0-1
    public static final double DriveTrainTrajMaxSpd = 36; // m/s
    public static final double DriveTrainTrajMaxAccel = 1; // m/s/s
    public static final double DriveTrainTrajMaxRotPctOutput = 0.25; // 0-1
    public static final double DriveTrainTrajMaxRotSpeed = 270 * Math.PI / 180; // rad/s
    public static final double DriveTrainTrajMaxRotAccel = 180 * Math.PI / 180; // rad/s^2
    public static final double DriveSpeedMultiplier = 4; //NOT RECOMENDED TO USE

    public static final class DriveTrajectoryPID {
      public static final double Pgain = 3.0;
      public static final double Igain = 0;
      public static final double Dgain = 0.0;
      public static final double PositionTolerance = 0.01; // m
      public static final double VelocityTolerance = 0.05; // m/s
    }

    public static final class DriveTrajectoryFF {
      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public static final class RotationTrajectoryPID {
      public static final double Pgain = 0.12;
      public static final double Igain = 0;
      public static final double Dgain = 0.0;
      public static final double PositionTolerance = 2.0; // Deg
      public static final double VelocityTolerance = 4.0; // Deg/s
    }

    public static final class RotationTrajectoryFF {
      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
    }
  }
}
