// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToLowerRung extends SequentialCommandGroup {
  /** Creates a new ClimbToLowerRung. */
  public ClimbToLowerRung() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    var climberPivot = RobotContainer.climberElevatorPivot;
    var climberElevator = RobotContainer.climberElevator;
    var climberShortPivot = RobotContainer.climberShortArmPivot;
    // TODO: add intake stow command
    addCommands(
        // ready
        climberPivot.goToIntermediateCommand(),
        climberShortPivot.goToReadyCommand(),
        climberPivot.goToReadyCommand(),
        climberElevator.goToTopCommand(),
        // engage upper hooks
        climberPivot.goToEngagedCommand(),
        // pull
        climberElevator.goToBottomCommand(),
        // engage lower hooks
        climberShortPivot.goToEngagedCommand());
  }
}
