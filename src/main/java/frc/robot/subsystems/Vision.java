// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Vision extends SubsystemBase {
  private String limelight = "limelight";
  private Pose2d botpose;

  /** Creates a new Vision. */
  public Vision() {
  }

  public Vision(String name) {
    this.limelight = name;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(this.limelight + "vision: id", LimelightHelpers.getFiducialID(this.limelight));

    botpose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
    // var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    var targetCount = LimelightHelpers.getTargetCount(limelight);
    if (botpose != null && botpose.getX() != 0 && botpose.getY() != 0 && targetCount > 0) {
      SmartDashboard.putNumber(this.limelight + "vision:botpose:blue:x", botpose.getX());
      SmartDashboard.putNumber(this.limelight + "vision:botpose:blue:y", botpose.getY());
      SmartDashboard.putNumber(this.limelight + "vision:botpose:blue:rot", botpose.getRotation().getDegrees());

      // RobotContainer.drivetrain.addVisionMeasurement(botpose,
      // Timer.getFPGATimestamp());
    }
  }

  // /**
  // * combines vision estamates by averaging them.
  // * will return null if not enough april tags are seen
  // *
  // * @param visionArr
  // * @param minimumTagCount
  // * @return
  // */
  public static Pose2d combineVisionEstemates(Vision[] visionArr, int minimumTagCount) {
    // List<PoseEstimate> estimates = new ArrayList<PoseEstimate>();
    int counter = 0;
    var xSum = 0.0;
    var ySum = 0.0;
    var rotSum = 0.0;
    for (int i = 0; i < visionArr.length; i++) {
      var vision = visionArr[i];
      // var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(vision.limelight);
      // if (estimate.tagCount >= minimumTagCount) {
      // estimates.add(estimate);
      // xSum += estimate.pose.getX();
      // ySum += estimate.pose.getY();
      // rotSum += estimate.pose.getRotation().getRadians();
      // }
      var tagCount = LimelightHelpers.getTargetCount(vision.limelight);
      if (tagCount >= minimumTagCount) {
        var estimate = LimelightHelpers.getBotPose2d_wpiBlue(vision.limelight);
        xSum += estimate.getX();
        ySum += estimate.getY();
        rotSum += estimate.getRotation().getRadians();
        // estimates.add(estimate);
        counter++;
      }
    }
    // if (estimates.size() == 0)
    // return null;
    SmartDashboard.putNumber("vision counter", counter);
    SmartDashboard.putNumber("vision xSum", xSum);

    if (counter == 0)
      return null;

    return new Pose2d(xSum / counter, ySum / counter,
        Rotation2d.fromRadians(rotSum / counter));
  }

  public static Pose2d combineVisionEstemates(Vision[] visionArr) {
    return combineVisionEstemates(visionArr, 2);
  }

  public double distanceToTargetInInches() {

    var target = LimelightHelpers.getBotPose_TargetSpace("limelight");

    if (target.length >= 2) {
      return target[2] * -39.3701;
    }
    return -1;
  }
}
