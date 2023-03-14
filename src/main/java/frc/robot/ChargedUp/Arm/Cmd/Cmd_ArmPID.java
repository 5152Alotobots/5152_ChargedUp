package frc.robot.ChargedUp.Arm.Cmd;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ChargedUp.Arm.Const_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;


public class Cmd_ArmPID extends CommandBase {

    private SubSys_Arm armSubSys;
    private DoubleSupplier rotationTarget;
    private DoubleSupplier extensionTarget;

    private boolean isFinished = false;
    private boolean isRotFinished;
    private boolean isExtFinished;

    public Cmd_ArmPID (SubSys_Arm armSubSys, DoubleSupplier rotationTarget, DoubleSupplier extensionTarget) {
        this.armSubSys = armSubSys;
        this.rotationTarget = rotationTarget;
        this.extensionTarget = extensionTarget;
        addRequirements(armSubSys);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Object armShoulderObject = armSubSys.setShoulderToPos(rotationTarget.getAsDouble());
        Object armExtendObject = armSubSys.setExtendToPos(extensionTarget.getAsDouble());
        
        if((boolean) (armShoulderObject = true)) {
            isRotFinished = true;
        } else {
            armSubSys.RotateArm(0, (double) armShoulderObject);
        }

        if((boolean) (armExtendObject = true)) {
            isExtFinished = true;
        }

        if (isRotFinished && isExtFinished) {
            isFinished = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
      
        return isFinished;
    }
}