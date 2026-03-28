// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Retention;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.FireAnimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.RobotBase;
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
  private Shooter2 shooter;
  private ShooterHood hood;
  private Loader loader;
  private Hopper hopper;
  private final double loaderSpeed = 0.8;

  class FiringSolution {
    public double distance;
    public double shooterSpeed;
    public double hoodPosition;
    public double airTime;

    public FiringSolution(double distance, double shooterSpeed, double hoodPosition, double airTime) {
      this.distance = distance;
      this.shooterSpeed = shooterSpeed;
      this.hoodPosition = hoodPosition;
      this.airTime = airTime;
    }
  };

  static class Vector2D {
    public double x;
    public double y;

    public Vector2D(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public Vector2D multiplyByDouble(double a) {
      this.x *= a;
      this.y *= a;
      return this;
    }

    public Vector2D clone() {
      return new Vector2D(x, y);
    }

    public Vector2D fromPose2D(Pose2d pose) {
      // return new Vector2D(pose.getX(), pose.getY());
      x = pose.getX();
      y = pose.getY();
      return this;
    }

    public Pose2d toPose2D() {
      return new Pose2d(x, y, new Rotation2d(0.0));
    }

    public Vector2D addVector(Vector2D v) {
      x += v.x;
      y += v.y;
      return this;
    }
  }

  FiringSolution[] solutions = new FiringSolution[] {
      // new FiringSolution(0, 46, 7),
      // new FiringSolution(1.00, 46, 7),
      // new FiringSolution(2.00, 46, 11.5),
      // new FiringSolution(2.7, 50, 14), // 2/26/26
      // new FiringSolution(3.12, 54, 18), // 2/26/26
      // new FiringSolution(3.34, 57, 17), // 2/26/26
      // new FiringSolution(4.14, 56, 23),
      // new FiringSolution(5.36, 62, 30),

      // new FiringSolution(0, 42, 3, 1),
      // new FiringSolution(0.8, 42, 4, 1),
      // new FiringSolution(1.23, 42, 8, 1),
      // new FiringSolution(2.6, 48, 20, 1), // good
      // new FiringSolution(3.45, 58, 18, 1),
      // new FiringSolution(3.65, 64, 19, 1),
      // new FiringSolution(5, 53, 17, 1),

      new FiringSolution(0.0, 34, 10, 1),
      new FiringSolution(1.36, 34, 10, 1), // 3/26/26
      new FiringSolution(1.85, 36, 12, 1.05), // 3/27/26
      new FiringSolution(2.35, 37, 14, 1.07), // 3/27/26
      new FiringSolution(2.86, 39, 17, 1.25), // 3/27/26
      new FiringSolution(3.37, 41, 20, 1.18), // 3/27/26
      new FiringSolution(3.90, 41.5, 23, 1.18), // 3/27/26
      new FiringSolution(4.75, 45, 23, 1.20), // 3/27/26
      new FiringSolution(10, 45, 23, 4),

  };

  public LaunchingTower(Shooter2 shooter, ShooterHood hood, Loader loader, Hopper hopper) {
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
      loader.setSpeed(loaderSpeed);
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
        (upper.hoodPosition * upperW + lower.hoodPosition * lowerW) / (lowerW + upperW),
        (upper.airTime * upperW + lower.airTime * lowerW) / (lowerW + upperW));
  }

  public Command shootCloseCommand() {
    return run(() -> {
      shootClose();
    }).finallyDo(() -> {
      shooter.setSpeed(0);
      hood.setPositionByPid(0);
      hopper.setSpeed(0);
      loader.setSpeed(0);
    });
  }

  public void shootClose() {
    shooter.setVelocity(10);
    hood.setPositionByPid(4);
    if (shooter.isAtTargetSpeed()) {
      loader.setSpeed(loaderSpeed);
      hopper.setSpeed(1);
    } else {
      loader.setSpeed(0);
      hopper.setSpeed(0);
    }
  }

  /**
   * returns the position that the robot should point towards
   * 
   * @return
   */
  public Vector2D getActualTarget() {
    var loops = 5;
    var velocity = new Vector2D(RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond,
        RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond);
    var solution = getSolution(RobotContainer.drivetrain.distanceToHub());
    var offset = velocity.clone().multiplyByDouble(-solution.airTime);
    var target = new Vector2D(0, 0).fromPose2D(RobotContainer.drivetrain.hubMidPoint).addVector(offset);

    for (int i = 0; i < loops; i++) {
      solution = getSolution(distanceToVector(target));
      offset = velocity.clone().multiplyByDouble(-solution.airTime);
      target = new Vector2D(0, 0).fromPose2D(RobotContainer.drivetrain.hubMidPoint).addVector(offset);
    }
    return target;
  }

  private double distanceToVector(Vector2D v) {
    var robotPose = RobotContainer.drivetrain.getState().Pose;
    return Math.sqrt(Math.pow(v.x - robotPose.getX(), 2) + Math.pow(v.y - robotPose.getY(), 2));
  }

  public Vector2D getVectorToHub() {
    var robotPose = RobotContainer.drivetrain.getState().Pose;
    var hubPose = RobotContainer.drivetrain.hubMidPoint;
    return new Vector2D(hubPose.getX() - robotPose.getX(), hubPose.getY() - robotPose.getY());
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
