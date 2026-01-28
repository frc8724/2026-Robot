// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoChooser extends SubsystemBase {
    public AutoChooser() {
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("Auto Wait", 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public void addAuto(Command cmd) {
        String name = cmd.getClass().getSimpleName();

        addAuto(name, cmd);
    }

    public void addAuto(String name, Command cmd) {
        autoChooser.addOption(name, cmd);
    }

    public Command getAutoCommand() {
        // run the auto command prefixed by a wait command
        // new SequentialCommandGroup(
        // new WaitCommand(SmartDashboard.getNumber("Auto Wait", 0)),
        // autoChooser.getSelected());
        return autoChooser.getSelected();
    }
}
