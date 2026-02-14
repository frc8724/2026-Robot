// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Retention;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameTimer extends SubsystemBase {
  boolean redWonAuto = true;

  /** Creates a new GameTimer. */
  public GameTimer() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean isAuto = DriverStation.isAutonomousEnabled();
    double matchTime = DriverStation.getMatchTime();
    boolean redHub = false;
    boolean blueHub = false;
    if (isAuto) {
      redWonAuto = Math.random() * 2 > 1 ? true : false;
    }

    if (matchTime > 130) {
      redHub = true;
      blueHub = true;
    } else if (matchTime > 105) {
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
    } else if (matchTime > 80) {
      if (!redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
    } else if (matchTime > 55) {
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
    } else if (matchTime > 30) {
      if (!redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
    } else {
      redHub = true;
      blueHub = true;
    }

    SmartDashboard.putBoolean("blue hub active", blueHub);
    SmartDashboard.putBoolean("red hub active", redHub);
    SmartDashboard.putNumber("game time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("red won auto", redWonAuto);
  }
}
