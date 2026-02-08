// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;

public class ClimberElevator extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX motor;
  private final PositionVoltage position = new PositionVoltage(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public ClimberElevator(TalonFX motor) {
    this.motor = motor;
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // configs.Slot0.kP = 4.0;
    // configs.Slot0.kD = 0.4;

    configs.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    configs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    configs.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    configs.Slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    configs.Slot0.kI = 0; // no output for integrated error
    configs.Slot0.kD = 0.4; // A velocity error of 1 rps results in 0.1 V output

    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // set Motion Magic settings
    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 4;
    motionMagicConfigs.MotionMagicAcceleration = 1;
    motionMagicConfigs.MotionMagicJerk = 100;

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
    }
  }

  public void setPosition(double pos) {
    if (motor != null) {
      // motor.setControl(position.withPosition(pos));
      motor.setControl(motionMagicRequest.withPosition(pos));
    }
  }

  Command SetPositiongInstantCommand(double pos) {
    return run(() -> {
      setPosition(pos);
    });
  }

  Command WaitUntilAtPositionCommand(double pos) {
    return run(() -> {
      if (this.isAtPosition(pos)) {

      }
    });
  }

  public Command setPositionCommand(double pos) {
    return run(() -> {
      setPosition(pos);
    }).until(() -> {
      return isAtPosition(pos);
    });
  }

  public void setPower(double d) {
    motor.set(d);
  }

  public Command setPowerCommand(double d) {
    return runOnce(() -> {
      setPower(d);
    });
  }

  public double getPosition() {
    return motor.getPosition(true).getValueAsDouble();
  }

  public boolean isAtPosition(double targetPos) {
    return Math.abs(targetPos - getPosition()) < .2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator:position", getPosition());
  }

  public Command controlWithAxis(DoubleSupplier axis) {
    return run(() -> {
      setPower(axis.getAsDouble() / 10);
    });
  }
}
