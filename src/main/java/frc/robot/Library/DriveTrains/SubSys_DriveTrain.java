/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Library.DriveTrains;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.DriveTrains.SwerveDrive.SubSys_SwerveDrive;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

public class SubSys_DriveTrain extends SubsystemBase {
  /**
   * Creates a new Drive SubSystem.
   */
  
  // Drive Types - Select only 1
  
  // Tank
  //private final TankDriveSubSys m_Drive = 
  //new TankDriveSubSys;

  // Mecanum
  //private final MecanumDriveSubSys m_Drive = 
  //new MecanumDriveSubSys;

  // Swerve
  private SubSys_SwerveDrive driveTrain; 

  // Drive Commands
  private double driveXDirCmd = 0;
  private double driveYDirCmd = 0;
  private double driveZRotCmd = 0;
  private boolean driveFieldOriented = false;
  private boolean driveRotateLeftPtCmd = false;
  private boolean driveRotateRightPtCmd = false;

  // GyroScope
  private SubSys_PigeonGyro gyroSubSys; 


  /**
   * SubSys_DriveTrain Constructor
   * @param gyroSubSys SubSys_PigeonGyro
   */
  public SubSys_DriveTrain(SubSys_PigeonGyro gyroSubSys) {
    this.gyroSubSys = gyroSubSys;
    this.driveTrain = new SubSys_SwerveDrive(this.gyroSubSys);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Send Drive Commands
    driveTrain.drive(
      new Translation2d(driveXDirCmd, driveYDirCmd),
      driveZRotCmd,
      driveFieldOriented,
      false,
      driveRotateLeftPtCmd,
      driveRotateRightPtCmd);
  }

  /***********************************************************************************/
  /* ***** Public DriveTrain Methods *****                                               */
  /***********************************************************************************/

  // ***** DriveTrain Info *****

  /** getMaxDriveSubSysSpd
   * Returns Max Drive SubSystem Speed
   * 
   * @return double DriveTrain Maximum Speed (m/s)
   */
  public double getMaxDriveSubSysSpd(){
    return SubSys_DriveTrain_Constants.DriveTrainMaxSpd;
  }

  /** getMaxDriveSubSysRotSpd
   * Returns Max Drive Subsystem Rotation
   * @return double DriveTrain Maximum Speed (rads/s)
   */
  public double getMaxDriveSubSysRotSpd(){
    return SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed;
  }

  // ***** Drive Methods *****
  
  /** Drive
  * Method to drive the robot using setpoint.
  *
  * @param xSpdCmd       Speed Cmd of the robot in the x direction (forward) m/s.
  * @param ySpdCmd       Speed Cmd of the robot in the y direction (sideways) m/s.
  * @param rotSpdCmd     Rotational speed Cmd of the robot. rad/s
  * @param fieldRelative Whether the provided x and y speeds are relative to the field.
  * @param rotateLeftPt  boolean Rotate around Left Pt
  * @param rotateRightPt boolean Rotate around Right Pt
  */
  @SuppressWarnings("ParameterName")
  public void Drive(
    double xSpdCmd,
    double ySpdCmd,
    double rotSpdCmd,
    boolean fieldRelative,
    boolean rotateLeftPtCmd,
    boolean rotateRightPtCmd) {
      
    //Limit Cmds to Chassis Limits
    driveXDirCmd = Math.min(Math.max(xSpdCmd,-SubSys_DriveTrain_Constants.DriveTrainMaxSpd),SubSys_DriveTrain_Constants.DriveTrainMaxSpd);
    driveYDirCmd = Math.min(Math.max(ySpdCmd,-SubSys_DriveTrain_Constants.DriveTrainMaxSpd),SubSys_DriveTrain_Constants.DriveTrainMaxSpd);
    driveZRotCmd = Math.min(Math.max(rotSpdCmd,-SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed),SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed);
    driveFieldOriented = fieldRelative;
    driveRotateLeftPtCmd = rotateLeftPtCmd;
    driveRotateRightPtCmd = rotateRightPtCmd;
  }

  // ***** Odometry *****
 
  /** getHeading
   * Get Swerve Drive Heading in Rotation2d
   * @return Rotation2d Heading of the drive train
   */
  public Rotation2d getHeading() {
      return this.driveTrain.getHeading();
  }

  /** setGyroYaw
   * set Gyro Yaw Value
   * @param degrees
   */
  public void setGyroYaw(double degrees){
    this.driveTrain.setGyroYaw(degrees);  
  }

  /** setGyroYawToZero
   * set Gyro Yaw Value to Zero
   */ 
  public void setGyroYawToZero(){
    this.driveTrain.setGyroYawToZero();  
  }

  /** getPose
   * Get the X and Y position of the drivetrain from the DriveTrain
   * @return Pose2d X and Y position of the drivetrain
   */
  public Pose2d getPose() {
    return this.driveTrain.getPose();
  }

  /** setPose
   * Set Pose of the drivetrain
   * @param pose Pose2d X and Y position of the drivetrain
   */
  public void setPose(Pose2d pose){
      this.driveTrain.setPose(pose);
  }
}
