// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchingTower extends SubsystemBase {
  /** Creates a new LaunchingTower. */
  private Shooter shooter;
  private ShooterHood hood;
  private Loader loader;
  private Hopper hopper;

  public LaunchingTower(Shooter shooter, ShooterHood hood, Loader loader, Hopper hopper) {
    this.shooter = shooter;
    this.hood = hood;
    this.loader = loader;
    this.hopper = hopper;
  }

  public Command prepareToFireAtCommand(double distance) {
    //
    return new ParallelCommandGroup(
        hood.SetPositiongByMMCommand(convertDistanceToHood(distance)).finallyDo(() -> {
          hood.setPositionByMM(0);
        }),
        shooter.setShooterSpeedCommand(convertDistanceToShooterRPM(distance)).finallyDo(() -> {
          shooter.setShooterSpeed(0);
        }));
  }

  public Command fireCommand(double distance) {
    return run(() -> {
      if (shooter.isAtTargetSpeed()) {
        loader.setSpeed(.5);
        hopper.setSpeed(.5);
      } else {
        loader.setSpeed(0);
        hopper.setSpeed(0);
      }
    }).finallyDo(() -> {
      loader.setSpeed(0);
      shooter.setShooterSpeed(0);
      hood.setPositionByMM(0);
      hopper.setSpeed(0);
    });
  }

  public double convertDistanceToShooterRPM(double distance) {
    return 0;
  }

  public double convertDistanceToHood(double distance) {
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
