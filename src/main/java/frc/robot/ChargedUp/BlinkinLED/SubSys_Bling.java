// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.BlinkinLED;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Bling extends SubsystemBase {
  private Spark BlinkinLED = new Spark(0);

  /** Creates a new SubSys_Blinkin. */
  public SubSys_Bling() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBlinkinLEDPurple() {
    // Sets PWM revlib spark max to 0.91
    BlinkinLED.set(0.91);
  }

  public void setBlinkinLEDYellow() {
    BlinkinLED.set(0.69);
  }

  public void setBlinkinLEDCustom(double PWMColorCode) {
    BlinkinLED.set(PWMColorCode);
  }
}
