// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  TalonFX motor;
  private final PositionVoltage position = new PositionVoltage(0);

  public IntakeArm(TalonFX motor) {
    this.motor = motor;
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kP = 4.0; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.4; // A change of 1 rotation per second results in 0.1 volts output

    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
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
  }

  public void setPosition(double pos) {
    if (motor != null) {
      motor.setControl(position.withPosition(pos));
    }
  }

  public Command setPositionCommand(double pos) {
    return runOnce(() -> {
      setPosition(pos);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
