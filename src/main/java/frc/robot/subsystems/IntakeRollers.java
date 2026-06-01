// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  /** Creates a new IntakeRollers. */
  private TalonFX motor;

  public IntakeRollers(TalonFX motor) {
    this.motor = motor;

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 30;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 30;
    motor.getConfigurator().apply(configs);
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

  public Command intakeCommand() {
    return setSpeedCommand(1);
  }

  public Command turnOffCommand() {
    return setSpeedCommand(0);
  }

  public Command outtakeCommand() {
    return setSpeedCommand(-.375);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
