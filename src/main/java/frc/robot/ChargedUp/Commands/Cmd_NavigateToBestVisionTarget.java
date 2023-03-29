/**
 * A command that navigates the robot to the best vision target using PhotonVision camera.
 * The robot is controlled based on the latest result obtained from the camera.
 * The command ends when the robot reaches the target.
 */
package frc.robot.ChargedUp.Commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.fasterxml.jackson.databind.deser.std.PrimitiveArrayDeserializers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings;

public class Cmd_NavigateToBestVisionTarget extends CommandBase {  
  //Declare Variables 
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Photonvision subSys_Photonvision;
  private final SubSys_Bling subSys_Bling;
  private final int pipelineIndex;
  private final PhotonCamera camera;
  private final boolean enableX;
  private final boolean enableRotation;
  
  private PhotonPipelineResult latestResult;
  /* X PID */
  private PIDController Xcontroller = new PIDController(DriveTrainTrajSettings.DriveTrajectoryPID.Pgain*3, DriveTrainTrajSettings.DriveTrajectoryPID.Igain, DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
  /* Y PID */
  private PIDController Zcontroller = new PIDController(DriveTrainTrajSettings.RotationTrajectoryPID.Pgain*1.4, DriveTrainTrajSettings.RotationTrajectoryPID.Igain, DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);

  /** Constructor
   *  @param subSys_DriveTrain The subsystem used for controlling the robot's drivetrain
   *  @param subSys_Photonvision The subsystem used for interfacing with the PhotonVision camera
   *  @param camera The PhotonVision camera to use
   *  @param pipelineIndex The index of the pipeline to use for targeting
   */
  public Cmd_NavigateToBestVisionTarget(SubSys_DriveTrain subSys_DriveTrain, SubSys_Photonvision subSys_Photonvision, SubSys_Bling subSys_Bling, PhotonCamera camera, int pipelineIndex, boolean enableRotation, boolean enableX) {
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Bling = subSys_Bling;
    this.pipelineIndex = pipelineIndex;
    this.camera = camera;
    this.enableRotation = enableRotation;
    this.enableX = enableX;
    addRequirements(subSys_DriveTrain, subSys_Photonvision, subSys_Bling);

    Xcontroller.setTolerance(0.01);
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
    latestResult = camera.getLatestResult();
    SmartDashboard.putBoolean("At setpoint X VISION", Xcontroller.atSetpoint());
    SmartDashboard.putBoolean("At setpoint Z VISION", Zcontroller.atSetpoint());
    SmartDashboard.putNumber("Error X PID VISION", Xcontroller.getPositionError());
    SmartDashboard.putNumber("Error Z PID VISION", Zcontroller.getPositionError());

    if (latestResult.hasTargets()){
    SmartDashboard.putNumber("Distance to Target", subSys_Photonvision.getRangeToTarget(latestResult));
    SmartDashboard.putNumber("Yaw to Target", latestResult.getBestTarget().getYaw());
    }

  // Rotation
  double rotSpd;
  if (enableRotation) {
   rotSpd = getVisionRotSpeed(latestResult);
  } else {
    rotSpd = 0;
  }
  // Forward/Backward
  double xSpd;
  if (enableX){
    xSpd = Math.min(Math.max(getVisionForwardSpeed(latestResult), -0.5), 0.5);
  } else {
    xSpd = 0;
  }
    subSys_DriveTrain.Drive(xSpd, 0, rotSpd, false, false, false);
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
    return (Xcontroller.getPositionError() < 0.04 && Xcontroller.getPositionError() > -0.04 && Zcontroller.getPositionError() < 1.1 && Zcontroller.getPositionError() > -1.1);
  }

  /** Calculate rotation speed */
  private double getVisionRotSpeed(PhotonPipelineResult result){
    if (result.hasTargets()) {
      return Zcontroller.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      return 0;
    }
  }

  /** Calculate forward speed */
  private double getVisionForwardSpeed(PhotonPipelineResult result){
    if (result.hasTargets()) {
      return -Xcontroller.calculate(subSys_Photonvision.getRangeToTarget(result), 0.45);
    } else {
      return 0;
    }
  }

  /** Returns true if the PID controllers don't need to move any further/you are at the target */
  private boolean isAtTarget(PhotonPipelineResult result){
    if (result.hasTargets()) {
      return Xcontroller.atSetpoint() && Zcontroller.atSetpoint();
    } else {
      return false;
    }
  }
}
