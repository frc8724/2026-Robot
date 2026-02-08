// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private TalonFX motor;

  /** Creates a new Hopper. */
  public Hopper(TalonFX motor) {
    this.motor = motor;
  }

  public void setSpeed(double speed) {
    if (motor != null) {
      motor.set(speed);
    }
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(() -> {
      setSpeed(speed);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
