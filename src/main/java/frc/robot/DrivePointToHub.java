// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivePointToHub extends Command {

  public static final double ROTATE_KP_RAD_PER_SEC_PER_DEG = 0.1 * 180;
  public static final double ROTATE_MAX_OMEGA_RAD_PER_SEC = 4.0;
  public static final double ROTATE_DEADBAND_RAD = Units.degreesToRadians(1);
  private int counter = 0;

  double angleRad;
  static int initCounter = 0;

  /** Creates a new DrivePointToHub. */
  public DrivePointToHub() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initCounter += 1;
    SmartDashboard.putNumber("point to hub init counter", initCounter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var angleRad = RobotContainer.drivetrain.robotOffsetAngleToHubRad();
    if (Math.abs(angleRad) > ROTATE_DEADBAND_RAD) {
      aimAtTarget();
      counter = 0;
    } else {
      counter += 1;
      if (counter > 4) {
        RobotContainer.drivetrain.lockWheels();
      } else {
        aimAtTarget();
      }
    }
  }

  public void aimAtTarget() {
    // boolean isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

    SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    angleRad = RobotContainer.drivetrain.robotOffsetAngleToHubRad();

    SmartDashboard.putString("debug", "" + angleRad);
    double omegaRadPerSec = 0.0;

    // if (Math.abs(angleRad) > ROTATE_DEADBAND_DEG) {
    omegaRadPerSec = MathUtil.clamp(
        ROTATE_KP_RAD_PER_SEC_PER_DEG * angleRad,
        -ROTATE_MAX_OMEGA_RAD_PER_SEC,
        ROTATE_MAX_OMEGA_RAD_PER_SEC);
    // }

    // Driver translation (robot-centric)
    double vx = 0.0;// vxMetersPerSec.getAsDouble();
    double vy = 0.0;// vyMetersPerSec.getAsDouble();

    RobotContainer.drivetrain.setControl(
        request.withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(omegaRadPerSec));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
