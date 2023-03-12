// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm.Cmd;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Arm.SubSys_Arm;



public class CmdGrp_MoveToMidPos extends ParallelCommandGroup {
  private final SubSys_Arm m_Arm;
  /** Creates a new CmdGrp_MoveToTopPos. */
  public CmdGrp_MoveToMidPos(SubSys_Arm armSubSys) {

    m_Arm = armSubSys;
    
    addCommands(
      new Cmd_ArmExtensionPID(armSubSys, 0),
      new Cmd_ArmRotationPID(armSubSys, 0)
    );
  }
}
