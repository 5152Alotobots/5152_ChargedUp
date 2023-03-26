// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Bling.Cmd;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ChargedUp.Bling.Const_Bling.Patterns;
import frc.robot.ChargedUp.Bling.SubSys_Bling;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_SetBlingColorAlliance extends InstantCommand {
  private SubSys_Bling subSys_Bling;
  private Spark controller;

  public Cmd_SetBlingColorAlliance(SubSys_Bling subSys_Bling, Spark Controller, double ColorValue) {
    this.subSys_Bling = subSys_Bling;
    this.controller = Controller;
    addRequirements(subSys_Bling);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        subSys_Bling.setBlinkinLEDColor(controller, Patterns.FixedPalette.BreathBlue);
      } else {
        subSys_Bling.setBlinkinLEDColor(controller, Patterns.FixedPalette.BreathRed);
      }
    }
  }
}
