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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("vision: id", LimelightHelpers.getFiducialID("limelight"));

    // var target = LimelightHelpers.getBotPose_TargetSpace("limelight");
    // SmartDashboard.putNumber("vision: x", target[0]);
    // SmartDashboard.putNumber("vision: y", target[1]);
    // SmartDashboard.putNumber("vision: z", target[2]);

    SmartDashboard.putNumber("vision: distance to target", this.distanceToTargetInInches());

    // LimelightHelpers.LimelightResults result =
    // LimelightHelpers.getLatestResults(limelight);
    // SmartDashboard.putString("vision:results:string", result.error);
    // var targetFiducials = result.targets_Fiducials;
    // SmartDashboard.putNumber("vision:result_length",
    // result.targets_Fiducials.length);
    // if (targetFiducials.length > 0) {
    // SmartDashboard.putNumber("vision:target:x", targetFiducials[0].tx);
    // SmartDashboard.putNumber("vision:target:y", targetFiducials[0].tx);
    // SmartDashboard.putNumber("vision:target;id", targetFiducials[0].fiducialID);
    // botpose = LimelightHelpers.getBotPose2d(limelight);
    // if (botpose != null) {
    // SmartDashboard.putNumber("vision:botpose:x", botpose.getX());
    // SmartDashboard.putNumber("vision:botpose:y", botpose.getY());
    // SmartDashboard.putNumber("vision:botpose:rot",
    // botpose.getRotation().getDegrees());

    // RobotContainer.drivetrain.addVisionMeasurement(
    // botpose,
    // Timer.getFPGATimestamp());
    // }
    botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
    if (botpose != null && botpose.getX() != 0 && botpose.getY() != 0) {
      SmartDashboard.putNumber("vision:botpose:blue:x", botpose.getX());
      SmartDashboard.putNumber("vision:botpose:blue:y", botpose.getY());
      SmartDashboard.putNumber("vision:botpose:blue:rot", botpose.getRotation().getDegrees());

      RobotContainer.drivetrain.addVisionMeasurement(botpose,
          Timer.getFPGATimestamp());
    }
    // }

  }

  public double distanceToTargetInInches() {

    var target = LimelightHelpers.getBotPose_TargetSpace("limelight");

    if (target.length >= 2) {
      return target[2] * -39.3701;
    }
    return -1;
  }
}
