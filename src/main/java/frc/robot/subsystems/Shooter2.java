// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter2 extends SubsystemBase {
  TalonFX motor1;
  TalonFX motor2;
  TalonFX motor3;
  TalonFX motor4;
  double currentSpeed = 0.0;
  double targetSpeed = 0.0;
  final VelocityVoltage m_request;
  double p = 1.0;
  double offset = 0.0;
  double idleSpeed = 0.0;

  /** Creates a new Shooter. */
  public Shooter2(TalonFX motor1, TalonFX motor2, TalonFX motor3, TalonFX motor4) {
    SmartDashboard.putNumber("shooter:P", p);
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.motor3 = motor3;
    this.motor4 = motor4;
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.0; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 20rps results in 0.2 V output
    slot0Configs.kP = 0.1;// 1.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.00; // no output for error derivative
    if (this.motor1 != null) {

      // shooterFx.getConfigurator().apply(slot0Configs);

      TalonFXConfiguration configs = new TalonFXConfiguration();
      configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      configs.Slot0 = slot0Configs;
      configs.Voltage.PeakReverseVoltage = 1;

      this.motor1.getConfigurator().apply(configs);
      // TODO: set alignments!!!
      if (this.motor2 != null) {
        this.motor2.setControl(new Follower(this.motor1.getDeviceID(), MotorAlignmentValue.Aligned));
      }
      if (this.motor3 != null) {
        this.motor3.setControl(new Follower(this.motor1.getDeviceID(), MotorAlignmentValue.Opposed));
      }
      if (this.motor4 != null) {
        this.motor4.setControl(new Follower(this.motor1.getDeviceID(), MotorAlignmentValue.Opposed));
      }
    }
    m_request = new VelocityVoltage(0).withSlot(0);
    setDefaultCommand(run(() -> {
      if (idleSpeed < 5) {
        setSpeed(0);
        idleSpeed = 0;
      } else {
        setVelocity(idleSpeed);
      }
    }));
  }

  public Command setIdleSpeedCommand(double idle) {
    return runOnce(() -> {
      idleSpeed = idle;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (motor1 != null) {
      SmartDashboard.putNumber("shooter velocity", motor1.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("shooter target", this.targetSpeed);
      SmartDashboard.putBoolean("shooter isAtTargetSpeed", isAtTargetSpeed());
    }
    SmartDashboard.putBoolean("shooter motor valid", motor1 != null);
  }

  public void addOffset(double o) {
    offset += o;
  }

  public Command addOffsetCommand(double o) {
    return runOnce(() -> {
      addOffset(o);
    });
  }

  public void setOffset(double o) {
    offset = o;
  }

  public Command setOffsetCommand(double o) {
    return runOnce(() -> {
      setOffset(o);
    });
  }

  public Command offsetShooterSpeedCommand(double i) {
    return runOnce(() -> {
      offsetShooterSpeed(i);
    });
  }

  void offsetShooterSpeed(double i) {
    if (motor1 != null) {
      currentSpeed += i;
      motor1.set(currentSpeed);
    }
  }

  public Command setSpeedCommand(double d) {
    return runOnce(() -> {
      motor1.set(d);
    });
  }

  public void setSpeed(double d) {
    motor1.set(d);
  }

  public void setVelocity(double d) {
    if (motor1 != null) {
      targetSpeed = d + offset;
      motor1.setControl(m_request.withVelocity(targetSpeed));
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
    if (motor1 != null) {
      currentSpeed = d;
      motor1.set(d);
    }
  }

  public Command offsetShooterVelocityCommand(double d) {
    return runOnce(() -> {
      offsetShooterVelocity(d);
    });
  }

  void offsetShooterVelocity(double d) {
    if (motor1 != null) {
      targetSpeed += d;
      motor1.setControl(m_request.withVelocity(targetSpeed));
    }
  }

  public boolean isAtTargetSpeed() {
    if (motor1 != null) {
      return Math.abs(motor1.getVelocity().getValueAsDouble() - targetSpeed) < .9;
    } else {
      return true;
    }
  }
}
