// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("vision: id", LimelightHelpers.getFiducialID("limelight"));

    var target = LimelightHelpers.getBotPose_TargetSpace("limelight");
    SmartDashboard.putNumber("vision: x", target[0]);
    SmartDashboard.putNumber("vision: y", target[1]);
    SmartDashboard.putNumber("vision: z", target[2]);

    SmartDashboard.putNumber("vision: distance to target", this.distanceToTargetInInches());

  }

  public double distanceToTargetInInches(){
    var target = LimelightHelpers.getBotPose_TargetSpace("limelight");
    return target[2] * 39.3701;
  }
}
