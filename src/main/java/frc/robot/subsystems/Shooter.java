// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

// import edu.wpi.first.wpilibj2.*;

public class Shooter extends SubsystemBase {
  TalonFX shooterFx = new TalonFX(20);
  double currentSpeed = 0.0;
  double targetSpeed = 0.0;
  final VelocityVoltage m_request;

  /** Creates a new Shooter. */
  public Shooter() {
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    shooterFx.getConfigurator().apply(slot0Configs);
    m_request = new VelocityVoltage(0).withSlot(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter velocity", shooterFx.getVelocity().getValueAsDouble());
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

  public void setControl(double d) {
    targetSpeed = d;
    shooterFx.setControl(m_request.withVelocity(targetSpeed));
  }

  public Command setContrlCommand(double d) {
    return runOnce(() -> {
      setControl(d);
    });
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

  public Command offsetShooterVelocityCommand(double d) {
    return runOnce(() -> {
      offsetShooterVelocity(d);
    });
  }

  void offsetShooterVelocity(double d) {
    targetSpeed += d;
    shooterFx.setControl(m_request.withVelocity(targetSpeed));
  }

  public boolean isAtTargetSpeed() {
    return Math.abs(shooterFx.getVelocity().getValueAsDouble() - targetSpeed) < targetSpeed * .1;
  }
}
