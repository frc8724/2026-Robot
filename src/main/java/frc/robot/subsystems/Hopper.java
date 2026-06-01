// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Hopper extends SubsystemBase {
  final double minSpeed = -.5;
  final double maxSpeed = .75;
  private TalonFX motor;

  /** Creates a new Hopper. */
  public Hopper(TalonFX motor) {
    this.motor = motor;

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    this.motor.getConfigurator().apply(configs);
  }

  private double clamp(double speed) {
    return Math.min(maxSpeed, Math.max(minSpeed, speed));
  }

  public void setSpeed(double speed) {
    if (motor != null) {
      motor.set(clamp(speed));
    }
  }

  public Command setSpeedCommand(double speed) {
    return runOnce(() -> {
      setSpeed(speed);
    });
  }

  public Command turnOnCommand() {
    return setSpeedCommand(.75);
  }

  public Command turnOffCommand() {
    return setSpeedCommand(0);
  }

  public Command reverseCommand() {
    return setSpeedCommand(-.5);
  }

  public Command jiggleWiggleCommand() {
    return new SequentialCommandGroup(
        reverseCommand(),
        new WaitCommand(0.5),
        turnOffCommand(),
        new WaitCommand(0.25),
        turnOnCommand(),
        run(() -> {
        }) // this command never ends interupt to stop
    ).finallyDo(() -> {
      setSpeed(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
