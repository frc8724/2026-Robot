// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.FireAnimation;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LaunchingTower extends SubsystemBase {
  /** Creates a new LaunchingTower. */
  private Shooter shooter;
  private ShooterHood hood;
  private Loader loader;
  private Hopper hopper;

  class FiringSolution {
    public double distance;
    public double shooterSpeed;
    public double hoodPosition;

    public FiringSolution(double distance, double shooterSpeed, double hoodPosition) {
      this.distance = distance;
      this.shooterSpeed = shooterSpeed;
      this.hoodPosition = hoodPosition;
    }
  };

  FiringSolution[] solutions = new FiringSolution[] {
      // new FiringSolution(0, 46, 7),
      // new FiringSolution(1.00, 46, 7),
      // new FiringSolution(2.00, 46, 11.5),
      // new FiringSolution(2.7, 50, 14), // 2/26/26
      // new FiringSolution(3.12, 54, 18), // 2/26/26
      // new FiringSolution(3.34, 57, 17), // 2/26/26
      // new FiringSolution(4.14, 56, 23),
      // new FiringSolution(5.36, 62, 30),
      new FiringSolution(0, 42, 4),
      new FiringSolution(0.8, 42, 4),
      new FiringSolution(1.23, 42, 8),
      new FiringSolution(2.6, 46, 20), // good
      new FiringSolution(3.45, 56, 18),
      new FiringSolution(5, 53, 17),
  };

  public LaunchingTower(Shooter shooter, ShooterHood hood, Loader loader, Hopper hopper) {
    this.shooter = shooter;
    this.hood = hood;
    this.loader = loader;
    this.hopper = hopper;
  }

  private Command prepareToFireAtCommand() {
    DoubleSupplier dub1 = () -> {
      var distance = RobotContainer.drivetrain.distanceToHub();
      return convertDistanceToHood(distance);
    };
    DoubleSupplier dub2 = () -> {
      var distance = RobotContainer.drivetrain.distanceToHub();
      return convertDistanceToShooterRPM(distance);
    };
    return new ParallelCommandGroup(
        // hood.SetPositiongByMMCommand(dub1),
        hood.SetPositionByPidCommand(dub1),
        shooter.setVelocityCommand(dub2));
  }

  private Command fireCommand() {
    return new ParallelCommandGroup(fireLoaderShooterHoodCommand());
  }

  private Command fireLoaderShooterHoodCommand() {
    return run(() -> {
      var currentDistance = RobotContainer.drivetrain.distanceToHub();
      // Temporarily always feed (test mode)
      loader.setSpeed(1);
      hopper.setSpeed(1);

      hood.setPositionByPid(convertDistanceToHood(currentDistance));
      shooter.setVelocity(convertDistanceToShooterRPM(currentDistance));
    }).finallyDo(() -> {
      loader.setSpeed(0);
      // shooter.setVelocity(0);
      shooter.setSpeed(0);
      hood.setPositionByPid(0);
      hopper.setSpeed(0);
    });
  }

  public Command fireFuelCommand() {
    return new DeferredCommand(() -> {
      return new SequentialCommandGroup(
          prepareToFireAtCommand(),
          fireCommand());
    }, new HashSet<Subsystem>(Arrays.asList(hood, shooter, loader, hopper)));
  }

  public double convertDistanceToShooterRPM(double distance) {
    // return 50;
    return getSolution(distance).shooterSpeed;
  }

  public double convertDistanceToHood(double distance) {
    // return 15;
    return getSolution(distance).hoodPosition;
  }

  public FiringSolution getSolution(double distance) {
    FiringSolution lower = null;
    FiringSolution upper = null;
    for (int i = solutions.length - 2; i >= 0; i--) {
      var sol = solutions[i];
      if (sol.distance < distance) {
        lower = sol;
        upper = solutions[i + 1];
        break;
      }
    }
    var upperW = (distance - lower.distance) / (upper.distance - lower.distance);
    var lowerW = 1 - upperW;
    return new FiringSolution(distance,
        (upper.shooterSpeed * upperW + lower.shooterSpeed * lowerW) / (lowerW + upperW),
        (upper.hoodPosition * upperW + lower.hoodPosition * lowerW) / (lowerW + upperW));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var solution = getSolution(RobotContainer.drivetrain.distanceToHub());
    SmartDashboard.putNumber("firing solution:distance", solution.distance);
    SmartDashboard.putNumber("firing solution:shooter_speed", solution.shooterSpeed);
    SmartDashboard.putNumber("firing solution:hood_position", solution.hoodPosition);
  }
}
