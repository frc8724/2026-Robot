// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  TalonFX motor;
  // private double down = -18;
  private double down = -21.5;
  private double up = 0;
  // private double up = -5;
  private final PositionVoltage position = new PositionVoltage(0);
  private double lastPosition;

  public IntakeArm(TalonFX motor) {
    this.motor = motor;
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.Slot0.kP = 0.5; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second results in 0.1 volts output

    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 6;
    configs.Voltage.PeakReverseVoltage = -6;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 20;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    lastPosition = getPosition();
  }

  public Command goToDownCommand() {
    // return setPositionCommand(down);
    return runOnce(() -> {
      setPosition(down);
    });
  }

  public Command goToUpCommand() {
    // return setPositionCommand(up);
    return runOnce(() -> {
      setPosition(up);
    });
  }

  public void setPosition(double pos) {
    if (motor != null) {
      motor.setControl(position.withPosition(pos));
    }
  }

  public Command jiggleCommand() {
    return new SequentialCommandGroup(
        goToUpCommand(),
        new WaitCommand(.5),
        goToDownCommand(),
        new WaitCommand(.5),
        goToUpCommand(),
        new WaitCommand(.5),
        goToDownCommand());
  }

  public Command setPositionCommand(double pos) {
    return run(() -> {
      setPosition(pos);
    }).until(() -> {
      lastPosition = getPosition();
      return isAtPosition(pos);
    });
  }

  public boolean isAtPosition(double targetPos) {
    return Math.abs(targetPos - getPosition()) < .1;
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
    if (motor == null) {
      return 0;
    }
    return motor.getPosition(true).getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm position", getPosition());
    SmartDashboard.putNumber("arm Last position", lastPosition);
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

  public Command controlWithAxis(DoubleSupplier axis) {
    return run(() -> {
      var pow = axis.getAsDouble();
      if (Math.abs(pow) < .1) {
        setPosition(lastPosition);
      } else {
        lastPosition = getPosition();
        setPower(axis.getAsDouble() / 10);
      }
    });
  }
}
