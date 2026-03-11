// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Retention;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

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
    boolean transistion = false;

    /**
     * TELEOP &
     * TRANSITION begins 2:20
     * ALLIANCE SHIFT starts 2:10
     * 1:45
     * 1:20
     * 0:55
     * END GAME begins 0:30
     * MATCH end 0:00
     */

    // if (isAuto) {
    // redWonAuto = Math.random() * 2 > 1 ? true : false;
    // }
    if (DriverStation.getGameSpecificMessage().length() > 0) {
      redWonAuto = DriverStation.getGameSpecificMessage().charAt(0) == 'R';
    } else {
      redWonAuto = true;
    }
    SmartDashboard.putString("game data", DriverStation.getGameSpecificMessage());

    if (matchTime > 135) {
      redHub = true;
      blueHub = true;
      // transistion = true;
    } else if (matchTime > 130) {// ending transition
      redHub = true;
      blueHub = true;
      transistion = true;
    } else if (matchTime > 110) { // first shift
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = false;
    } else if (matchTime > 105) {
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = true;
    } else if (matchTime > 85) {
      if (!redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = false;
    } else if (matchTime > 80) {
      if (!redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = true;
    } else if (matchTime > 60) {
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = false;
    } else if (matchTime > 55) {
      if (redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = true;
    } else if (matchTime > 35) {
      if (!redWonAuto) {
        redHub = false;
        blueHub = true;
      } else {
        blueHub = false;
        redHub = true;
      }
      transistion = false;
    } else {
      transistion = false;
      redHub = true;
      blueHub = true;
    }

    SmartDashboard.putBoolean("transition", transistion);
    SmartDashboard.putBoolean("blue hub active", blueHub);
    SmartDashboard.putBoolean("red hub active", redHub);
    SmartDashboard.putNumber("game time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("red won auto", redWonAuto);

    if (redHub && blueHub) {
      if (transistion) {
        RobotContainer.lights.set(LEDLights.PatternID.STROBE_WHITE);
      } else {
        RobotContainer.lights.set(LEDLights.PatternID.VIOLET);
      }
    } else if (redHub) {
      RobotContainer.lights.set(transistion ? LEDLights.PatternID.STROBE_RED : LEDLights.PatternID.RED);
    } else if (blueHub) {
      RobotContainer.lights.set(transistion ? LEDLights.PatternID.STROBE_BLUE : LEDLights.PatternID.BLUE);
    } else {
      RobotContainer.lights.set(LEDLights.PatternID.GREEN);
    }
  }
}
