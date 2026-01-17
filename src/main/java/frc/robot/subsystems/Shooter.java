// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.*;

public class Shooter extends SubsystemBase {
  TalonFX shooterFx = new TalonFX(20);
  double currentSpeed = 0.0;
  double targetSpeed = 0.0;

  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command offsetShooterSpeedCommand(double i) {
    return runOnce(() -> {
      offsetShooterSpeed(i);
    });
  }
  void offsetShooterSpeed(double i) {
    currentSpeed += i;
    shooterFx.set(currentSpeed);
  }
  public Command setShooterSpeedCommand(double d) {
        return runOnce(() -> {
            setShooterSpeed(d);
        });
    }
    void setShooterSpeed(double d) {
      currentSpeed = d;
      shooterFx.set(d);
    }
}
