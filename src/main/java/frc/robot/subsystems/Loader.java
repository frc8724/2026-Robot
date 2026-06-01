// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
  private TalonFX motor;

  /** Creates a new Loader. */
  public Loader(TalonFX motor) {
    this.motor = motor;

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    this.motor.getConfigurator().apply(configs);
  }

  public void setSpeed(double speed) {
    if (motor != null) {
      motor.set(speed);
    }
  }

  public Command turnOnCommand() {
    return setSpeedCommand(.5);
  }

  public Command turnOffCommand() {
    return setSpeedCommand(0);
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
