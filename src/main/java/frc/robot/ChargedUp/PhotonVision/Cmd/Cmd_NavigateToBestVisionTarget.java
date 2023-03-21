/**
 * A command that navigates the robot to the best vision target using PhotonVision camera.
 * The robot is controlled based on the latest result obtained from the camera.
 * The command ends when the robot reaches the target.
 */
package frc.robot.ChargedUp.PhotonVision.Cmd;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;

public class Cmd_NavigateToBestVisionTarget extends CommandBase {  
  //Declare Variables 
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Photonvision subSys_Photonvision;
  private final SubSys_Bling subSys_Bling;
  private final int pipelineIndex;
  private final PhotonCamera camera;
  
  private PhotonPipelineResult result;

  /** Constructor
   *  @param subSys_DriveTrain The subsystem used for controlling the robot's drivetrain
   *  @param subSys_Photonvision The subsystem used for interfacing with the PhotonVision camera
   *  @param camera The PhotonVision camera to use
   *  @param pipelineIndex The index of the pipeline to use for targeting
   */
  public Cmd_NavigateToBestVisionTarget(SubSys_DriveTrain subSys_DriveTrain, SubSys_Photonvision subSys_Photonvision, SubSys_Bling subSys_Bling, PhotonCamera camera, int pipelineIndex) {
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Bling = subSys_Bling;
    this.pipelineIndex = pipelineIndex;
    this.camera = camera;
    addRequirements(subSys_DriveTrain, subSys_Photonvision, subSys_Bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(pipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = camera.getLatestResult();
    
    // Use values to drive the robot
    subSys_DriveTrain.Drive(subSys_Photonvision.getVisionForwardSpeed(result)/4, subSys_Photonvision.getVisionStrafeSpeed(result)/4, 0/*subSys_Photonvision.getVisionRotSpeed(result)/4*/, false, false, false);
    subSys_Bling.setBlinkinLEDColor(Const_Bling.Controllers.controller1, Const_Bling.Patterns.FixedPalette.BreathBlue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    subSys_DriveTrain.Drive(0, 0, 0, false, false, false);
    subSys_Bling.setBlinkinLEDColor(Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Gold);
  }

  // Returns true when the command should end.  
  @Override
  public boolean isFinished() {
    // Check if the robot is at the target
    return subSys_Photonvision.isAtTarget(result);
  }
}
