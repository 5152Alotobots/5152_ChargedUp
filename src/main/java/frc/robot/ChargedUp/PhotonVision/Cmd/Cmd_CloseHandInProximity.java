// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.PhotonVision.Cmd;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;

public class Cmd_CloseHandInProximity extends CommandBase {
  SubSys_Photonvision subSys_Photonvision;
  SubSys_Hand subSys_Hand;
  
  PhotonCamera cameraFront = new PhotonCamera("OV5647");

  private PhotonPipelineResult resultFront;
  private Boolean isFinished = false;
  /** Creates a new Cmd_CloseClawInProx. */
  public Cmd_CloseHandInProximity(SubSys_Photonvision subSys_Photonvision, SubSys_Hand subSys_Hand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Hand = subSys_Hand;

    addRequirements(subSys_Photonvision, subSys_Hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* FRONT CAMERA */
    resultFront = cameraFront.getLatestResult();
    
    if (subSys_Photonvision.isInRange(resultFront)){
      subSys_Hand.CloseHand();
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
