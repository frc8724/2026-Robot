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
  private IntakeRollers rollers;

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
      new FiringSolution(0, 79.8, 17),
      new FiringSolution(5, 79.8, 17)
  };

  public LaunchingTower(Shooter shooter, ShooterHood hood, Loader loader, Hopper hopper, IntakeRollers rollers) {
    this.shooter = shooter;
    this.hood = hood;
    this.loader = loader;
    this.hopper = hopper;
    this.rollers = rollers;
  }

  private Command prepareToFireAtCommand() {
    DoubleSupplier dub1 = () -> {
      var distance = RobotContainer.ratatouille.distanceToHub();
      return convertDistanceToHood(distance);
    };
    DoubleSupplier dub2 = () -> {
      var distance = RobotContainer.ratatouille.distanceToHub();
      return convertDistanceToShooterRPM(distance);
    };
    return new ParallelCommandGroup(
        hood.SetPositiongByMMCommand(dub1),
        shooter.setVelocityCommand(dub2));
  }

  private Command fireCommand() {
    // return run(() -> {
    // var currentDistance = RobotContainer.drivetrain.distanceToHub();
    // if (shooter.isAtTargetSpeed()) {
    // loader.setSpeed(.75);
    // hopper.setSpeed(.75);
    // rollers.setSpeed(0.5);
    // } else {
    // loader.setSpeed(0);
    // hopper.setSpeed(0);
    // rollers.setSpeed(0.0);
    // }
    // hood.setPositionByMM(convertDistanceToHood(currentDistance));
    // shooter.setVelocity(convertDistanceToShooterRPM(currentDistance));
    // }).finallyDo(() -> {
    // loader.setSpeed(0);
    // shooter.setShooterSpeed(0);
    // hood.setPositionByMM(0);
    // hopper.setSpeed(0);
    // rollers.setSpeed(0.0);
    // });
    return new ParallelCommandGroup(fireLoaderShooterHoodCommand()
    // ,
    // hopper.jiggleWiggleCommand()
    );
  }
/*
  private Command fireLoaderShooterHoodCommand() {
    return run(() -> {
      var currentDistance = RobotContainer.ratatouille.distanceToHub();
      if (shooter.isAtTargetSpeed()) {
        loader.setSpeed(1);
        // rollers.setSpeed(0.5);
        hopper.setSpeed(1);
      } else {
        loader.setSpeed(0);
        rollers.setSpeed(0.0);
        hopper.setSpeed(0);
      }
      hood.setPositionByMM(convertDistanceToHood(currentDistance));
      shooter.setVelocity(convertDistanceToShooterRPM(currentDistance));
    }).finallyDo(() -> {
      loader.setSpeed(0);
      // shooter.setShooterSpeed(0);
      shooter.setVelocity(0);

      hood.setPositionByMM(0);
      rollers.setSpeed(0.0);
      hopper.setSpeed(0);
    });
  }
*/
private Command fireLoaderShooterHoodCommand() {
    return run(() -> {
      var currentDistance = RobotContainer.ratatouille.distanceToHub();
      // Temporarily always feed (test mode)
      loader.setSpeed(1);
      hopper.setSpeed(1);
      
      hood.setPositionByMM(convertDistanceToHood(currentDistance));
      shooter.setVelocity(convertDistanceToShooterRPM(currentDistance));
    }).finallyDo(() -> {
      loader.setSpeed(0);
      shooter.setVelocity(0);
      hood.setPositionByMM(0);
      rollers.setSpeed(0.0);
      hopper.setSpeed(0);
    });
  }
  
  public Command fireFuelCommand() {
    return new DeferredCommand(() -> {
      var distance = RobotContainer.ratatouille.distanceToHub();
      return new SequentialCommandGroup(
          prepareToFireAtCommand(),
          fireCommand());
    }, new HashSet<Subsystem>(Arrays.asList(hood, shooter, loader, hopper, rollers)));
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
    var solution = getSolution(RobotContainer.ratatouille.distanceToHub());
    SmartDashboard.putNumber("firing solution:distance", solution.distance);
    SmartDashboard.putNumber("firing solution:shooter_speed", solution.shooterSpeed);
    SmartDashboard.putNumber("firing solution:hood_position", solution.hoodPosition);
  }
}
