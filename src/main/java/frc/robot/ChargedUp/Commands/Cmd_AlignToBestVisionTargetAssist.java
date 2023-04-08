/**
 * A command that navigates the robot to the best vision target using PhotonVision camera.
 * The robot is controlled based on the latest result obtained from the camera.
 * The command ends when the robot reaches the target.
 */
package frc.robot.ChargedUp.Commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision.AcceptablePIDError;
import frc.robot.ChargedUp.PhotonVision.Const_Photonvision.PIDspeeds;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings;

public class Cmd_AlignToBestVisionTargetAssist extends CommandBase {  
  //Declare Variables 
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Photonvision subSys_Photonvision;
  private final SubSys_Bling subSys_Bling;
  private final int pipelineIndex;
  private final PhotonCamera camera;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;
  
  private PhotonPipelineResult latestResult;
  /* Z PID */
  private PIDController Zcontroller = new PIDController(DriveTrainTrajSettings.RotationTrajectoryPID.Pgain*2.6, DriveTrainTrajSettings.RotationTrajectoryPID.Igain, DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);

  /** Constructor
   *  @param subSys_DriveTrain The subsystem used for controlling the robot's drivetrain
   *  @param subSys_Photonvision The subsystem used for interfacing with the PhotonVision camera
   *  @param camera The PhotonVision camera to use
   *  @param pipelineIndex The index of the pipeline to use for targeting
   *  @param xSpeed The speed to use for forward/backward movement
   *  @param ySpeed The speed to use for strafing 
   */
  public Cmd_AlignToBestVisionTargetAssist(SubSys_DriveTrain subSys_DriveTrain, SubSys_Photonvision subSys_Photonvision, SubSys_Bling subSys_Bling, PhotonCamera camera, int pipelineIndex, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Bling = subSys_Bling;
    this.pipelineIndex = pipelineIndex;
    this.camera = camera;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(subSys_DriveTrain, subSys_Photonvision, subSys_Bling);

    Zcontroller.setTolerance(0.3);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(pipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the latest result from the camera
    latestResult = camera.getLatestResult();
    SmartDashboard.putBoolean("At setpoint Z VISION", Zcontroller.atSetpoint());
    SmartDashboard.putNumber("Error Z PID VISION", Zcontroller.getPositionError());

    subSys_DriveTrain.Drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), getVisionRotSpeed(latestResult), true, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    subSys_DriveTrain.Drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.  
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Calculate rotation speed */
  private double getVisionRotSpeed(PhotonPipelineResult result){
    if (result.hasTargets()) {
      return Math.min(Math.max(Zcontroller.calculate(result.getBestTarget().getYaw(), 0), -PIDspeeds.Max_Z_PID_Speed), PIDspeeds.Max_Z_PID_Speed);
    } else {
      return 0;
    }
  }
}
