// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Library.DriveTrains.SwerveDrive.SwerveModules.MK4i_FalconFalcon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;

/** Swerve and Steer Module
 *      Falcon Drive Motor
 *      PG71 Steer Motor
 */
public final class MK4i_FalconFalcon_Module_Constants {
    
    public static TalonFXConfiguration DriveTalonFXConfig;
    public static TalonFXConfiguration SteerTalonFXConfig;

    public MK4i_FalconFalcon_Module_Constants(){      

        /** Swerve Drive Motor Configuration */
        DriveTalonFXConfig = new TalonFXConfiguration();

        DriveTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        
        DriveTalonFXConfig.slot0.kP = 0.0;
        DriveTalonFXConfig.slot0.kI = 0.0;
        DriveTalonFXConfig.slot0.kD = 0.0;
        DriveTalonFXConfig.slot0.kF = 0.0;        
        
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 
            35, 
            60, 
            0.1);
        
        DriveTalonFXConfig.supplyCurrLimit = driveSupplyLimit;
        
        DriveTalonFXConfig.openloopRamp = 0.25;
        DriveTalonFXConfig.closedloopRamp = 0.0;

        /** Steer Motor Configuration */
        SteerTalonFXConfig = new TalonFXConfiguration();

        SteerTalonFXConfig.slot0.kP = 0.6;
        SteerTalonFXConfig.slot0.kI = 0.0;
        SteerTalonFXConfig.slot0.kD = 0.0;
        SteerTalonFXConfig.slot0.kF = 0.0;
        
        SupplyCurrentLimitConfiguration steerSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 
            35, 
            60, 
            0.1);

        SteerTalonFXConfig.supplyCurrLimit = steerSupplyLimit;
        
        SteerTalonFXConfig.openloopRamp = 0.25;
        SteerTalonFXConfig.closedloopRamp = 0.0;
    }

    public static final class DriveMotor {

        /** Neutral Mode */
        public static final NeutralMode neutralMode = NeutralMode.Coast; 

        /** Drive Motor Wheel Circumference  */
        public static final double driveWheelDiameter = Units.inchesToMeters(4.0);  // meters
        public static final double driveWheelCircumference = driveWheelDiameter * Math.PI; // meters

        /** Drive Motor Gear Ratio */
        public static final double driveGearRatio = (6.75);             // 6.75:1
        public static final double invDriveGearRatio = (0.148148148);   // 1:6.75

        /** Feedforward Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);
    }

    public static final class SteerMotor {

        /** Neutral Mode */
        public static final NeutralMode neutralMode = NeutralMode.Coast; 

        /** Steer Motor Gear Ratio */
        public static final double steerGearRatio = (6.0 / 1.0);  // 6.0:1

        /** Internal Steer Encoder */
        public static final FeedbackDevice selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        public static final double selectedFeedbackCoefficient = 1;
        public static final boolean sensorPhase = false;
        public static final boolean feedbackNotContinuous = true;
        
        /** Motor Feedback Counts Per Motor Revolution */
        public static final int sensorCntsPerRev = 1024;
    }

    public static final class SteerCANcoder {
        public static final boolean sensorPhase = true;
        public static final double selectedFeedbackCoefficient = 1;
    }
}
