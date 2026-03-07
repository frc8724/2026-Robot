// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import edu.wpi.first.wpilibj2.*;

public class Shooter extends SubsystemBase {
  TalonFX shooterFx;
  TalonFX secondaryFx;
  double currentSpeed = 0.0;
  double targetSpeed = 0.0;
  final VelocityVoltage m_request;
  double p = 1.0;

  /** Creates a new Shooter. */
  public Shooter(TalonFX motor, TalonFX secondary) {
    SmartDashboard.putNumber("shooter:P", p);
    this.shooterFx = motor;
    this.secondaryFx = secondary;
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.0; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 20rps results in 0.2 V output
    slot0Configs.kP = 0.1;// 1.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.00; // no output for error derivative
    if (this.shooterFx != null) {

      // shooterFx.getConfigurator().apply(slot0Configs);

      TalonFXConfiguration configs = new TalonFXConfiguration();
      configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      configs.Slot0 = slot0Configs;
      configs.Voltage.PeakReverseVoltage = 1;

      this.shooterFx.getConfigurator().apply(configs);
      if (this.secondaryFx != null) {
        this.secondaryFx.setControl(new Follower(this.shooterFx.getDeviceID(), MotorAlignmentValue.Opposed));
      }
    }
    m_request = new VelocityVoltage(0).withSlot(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shooterFx != null) {
      SmartDashboard.putNumber("shooter velocity", shooterFx.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("shooter target", this.targetSpeed);
      SmartDashboard.putBoolean("shooter isAtTargetSpeed", isAtTargetSpeed());
    }
    SmartDashboard.putBoolean("shooter motor valid", shooterFx != null);
  }

  public Command offsetShooterSpeedCommand(double i) {
    return runOnce(() -> {
      offsetShooterSpeed(i);
    });
  }

  void offsetShooterSpeed(double i) {
    if (shooterFx != null) {
      currentSpeed += i;
      shooterFx.set(currentSpeed);
    }
  }

  public Command setSpeedCommand(double d) {
    return runOnce(() -> {
      shooterFx.set(d);
    });
  }

  public void setSpeed(double d) {
    shooterFx.set(d);
  }

  public void setVelocity(double d) {
    if (shooterFx != null) {
      targetSpeed = d;
      shooterFx.setControl(m_request.withVelocity(targetSpeed));
    }
  }

  public Command setVelocityCommand(DoubleSupplier d) {
    // return runOnce(() -> {
    // setVelocity(d);
    // });\
    return run(() -> {
      setVelocity(d.getAsDouble());
    }).until(() -> {
      return isAtTargetSpeed();
    });
  }

  public Command setVelocityCommand(double d) {
    // return runOnce(() -> {
    // setVelocity(d);
    // });\
    return run(() -> {
      setVelocity(d);
    }).until(() -> {
      return isAtTargetSpeed();
    });
  }

  public Command setShooterVbusCommand(double d) {
    return runOnce(() -> {
      setShooterSpeed(d);
    });
  }

  void setShooterSpeed(double d) {
    if (shooterFx != null) {
      currentSpeed = d;
      shooterFx.set(d);
    }
  }

  public Command offsetShooterVelocityCommand(double d) {
    return runOnce(() -> {
      offsetShooterVelocity(d);
    });
  }

  void offsetShooterVelocity(double d) {
    if (shooterFx != null) {
      targetSpeed += d;
      shooterFx.setControl(m_request.withVelocity(targetSpeed));
    }
  }

  public boolean isAtTargetSpeed() {
    if (shooterFx != null) {
      return Math.abs(shooterFx.getVelocity().getValueAsDouble() - targetSpeed) < .9;
    } else {
      return true;
    }
  }
}
