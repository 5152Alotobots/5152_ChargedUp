// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.PhotonVision.Cmd;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;

public class Cmd_NavigateToBestVisionTarget extends CommandBase {  
  //Declare Variables 
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Photonvision subSys_Photonvision;
  // Change this to match the name of your camera

  PhotonCamera cameraFront = new PhotonCamera("OV5647");
  // PhotonCamera cameraRear = new PhotonCamera("OV5647");
  
  private PhotonPipelineResult resultFront;

  /**Constructor 
   * @param TestFalconSubsystem
   * @param Joystick_DS 
   */
  public Cmd_NavigateToBestVisionTarget( SubSys_DriveTrain subSys_DriveTrain, SubSys_Photonvision subSys_Photonvision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Photonvision = subSys_Photonvision;
    addRequirements(subSys_DriveTrain, subSys_Photonvision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    /* FRONT CAMERA */
    resultFront = cameraFront.getLatestResult();
    
    //Use values to drive robot
    subSys_DriveTrain.Drive(subSys_Photonvision.getVisionForwardSpeed(resultFront), subSys_Photonvision.getVisionStrafeSpeed(resultFront), subSys_Photonvision.getVisionRotSpeed(resultFront), false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSys_DriveTrain.Drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.  
  @Override
  public boolean isFinished() {
    return subSys_Photonvision.isAtTarget(resultFront);
  }
}