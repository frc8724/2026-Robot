// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  private final String limelight = "limelight";
  private Pose2d botpose;

  /** Creates a new Vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("vision: id", LimelightHelpers.getFiducialID("limelight"));
    SmartDashboard.putNumber("vision: distance to target", this.distanceToTargetInInches());

    botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
    var targetCount = LimelightHelpers.getTargetCount(limelight);
    if (botpose != null && botpose.getX() != 0 && botpose.getY() != 0 && targetCount > 1) {
      SmartDashboard.putNumber("vision:botpose:blue:x", botpose.getX());
      SmartDashboard.putNumber("vision:botpose:blue:y", botpose.getY());
      SmartDashboard.putNumber("vision:botpose:blue:rot", botpose.getRotation().getDegrees());

      // RobotContainer.drivetrain.addVisionMeasurement(botpose,
      // Timer.getFPGATimestamp());
    }
  }

  public double distanceToTargetInInches() {

    var target = LimelightHelpers.getBotPose_TargetSpace("limelight");

    if (target.length >= 2) {
      return target[2] * -39.3701;
    }
    return -1;
  }
}
