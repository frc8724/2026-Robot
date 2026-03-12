// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
  private TalonFX motor;
  private final PositionVoltage position = new PositionVoltage(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  public final double min = 0;
  public final double max = 30;
  private double target;

  private double counter = 0;
  private final double tolerance = .1;
  private final double toloeranceCounter = 5;

  /** Creates a new ShooterHood. */
  public ShooterHood(TalonFX motor) {
    this.motor = motor;
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.Slot0.kS = 0.0; // Add 0.1 V output to overcome static friction
    configs.Slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    configs.Slot0.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    configs.Slot0.kP = 3.0; // A position error of 2.5 rotations results in 12 V output
    configs.Slot0.kI = .0; // no output for integrated error
    configs.Slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // set Motion Magic settings
    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5000;
    motionMagicConfigs.MotionMagicAcceleration = 5000;
    motionMagicConfigs.MotionMagicJerk = 10000;

    if (motor != null) {
      /* Retry config apply up to 5 times, report if failure */
      StatusCode status = StatusCode.StatusCodeNotInitialized;
      for (int i = 0; i < 5; ++i) {
        status = motor.getConfigurator().apply(configs);
        if (status.isOK())
          break;
      }
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }
      // motor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void setPositionByMM(double pos) {
    if (motor != null) {
      // motor.setControl(position.withPosition(pos));
      target = pos;
      motor.setControl(motionMagicRequest.withPosition(clamp(pos)));
    }
  }

  public void setPositionByPid(double pos) {
    if (motor != null) {
      target = pos;
      motor.setControl(position.withPosition(clamp(pos)));
    }
  }

  public Command SetPositiongByMMCommand(DoubleSupplier posSupplier) {
    return run(() -> {
      setPositionByMM(posSupplier.getAsDouble());
    }).until(() -> {
      return isAtPosition(posSupplier.getAsDouble());
    });
  }

  public Command SetPositiongByMMCommand(double pos) {
    return run(() -> {
      setPositionByMM(pos);
    }).until(() -> {
      return isAtPosition(pos);
    });
  }

  public Command SetPositionByPidCommand(DoubleSupplier posSupplier) {
    return run(() -> {
      setPositionByPid(posSupplier.getAsDouble());
    }).until(() -> {
      return isAtPosition(posSupplier.getAsDouble());
    });
  }

  public Command SetPositionByPidCommand(double pos) {
    return run(() -> {
      setPositionByPid(pos);
    }).until(() -> {
      return isAtPosition(pos);
    });
  }

  public Command setPositionCommand(double pos) {
    return run(() -> {
      setPositionByMM(pos);
    }).until(() -> {
      return isAtPosition(pos);
    });
  }

  private double clamp(double pos) {
    return Math.min(max, Math.max(min, pos));
  }

  public Command offsetPositionCommand(double offset) {
    return defer(() -> {
      this.target += offset;
      return setPositionCommand(this.target);
    });
  }

  // public Command offsetPositionInstantCommand(double offset) {
  // return runOnce(() -> {
  // offsetPositionCommand(offset);
  // });
  // }

  public void setPower(double d) {
    motor.set(d);
  }

  public Command setPowerCommand(double d) {
    return runOnce(() -> {
      setPower(d);
    });
  }

  public double getPosition() {
    if (motor == null) {
      return 0;
    }
    return motor.getPosition(true).getValueAsDouble();
  }

  public boolean isAtPosition(double targetPos) {
    // return Math.abs(targetPos - getPosition()) < .02;
    return (Math.abs(getPosition() - targetPos) < tolerance) && counter >= toloeranceCounter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood position", getPosition());
    SmartDashboard.putNumber("hood target", target);
    if (Math.abs(getPosition() - target) < tolerance) {
      counter++;
    } else {
      counter = 0;
    }
    SmartDashboard.putNumber("hood counter", counter);
    SmartDashboard.putBoolean("hood at position", isAtPosition(target));
    SmartDashboard.putNumber("hood x", motor.getClosedLoopError(true).getValueAsDouble());
  }

  public Command controlWithAxis(DoubleSupplier axis) {
    return run(() -> {
      setPower(axis.getAsDouble() / 10);
    });
  }

  public void zero() {
    if (motor == null) {
      return;
    }
    motor.setPosition(0);
  }

  public Command zeroCommand() {
    return runOnce(() -> {
      zero();
    });
  }
}
