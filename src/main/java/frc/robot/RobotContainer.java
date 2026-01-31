// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemExtreme3dPro.Axis;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
        private double MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // private final CommandXboxController joystick = new CommandXboxController(0);
        private final MayhemExtreme3dPro driverStick = new MayhemExtreme3dPro(1);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final AutoChooser m_auto = new AutoChooser();
        private static final Vision vision = new Vision();

        public RobotContainer() {

                configureBindings();

                m_auto.addAuto("auto drive shoot", new PathPlannerAuto("auto drive shoot"));
                m_auto.addAuto("Drive Back", new PathPlannerAuto("Drive Back"));
                m_auto.addAuto("Triangle", new PathPlannerAuto("Triangle"));
                m_auto.addAuto("Drive Side to Side", new PathPlannerAuto("Drive Side to Side"));
                m_auto.addAuto("Start Corner Shoot", new PathPlannerAuto("Start Corner Shoot"));
                m_auto.addAuto("Test X", new PathPlannerAuto("Test X"));
                m_auto.addAuto("Shoot Center Climb", new PathPlannerAuto("Shoot Center Climb"));

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() ->
                                // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                // negative Y (forward)
                                // .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
                                // (left)
                                // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                // counterclockwise with negative X (left)
                                drive.withVelocityX(-driverStick.getRawAxis(Axis.Y) * MaxSpeed) // Drive forward with
                                                                                                // negative Y
                                                                                                // (forward)
                                                .withVelocityY(-driverStick.getRawAxis(Axis.X) * MaxSpeed) // Drive left
                                                                                                           // with
                                                                                                           // negative X
                                                                                                           // (left)
                                                .withRotationalRate(-driverStick.getRawAxis(Axis.Z) * MaxAngularRate) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                                ));

                driverStick.Button(11).onTrue(drivetrain.zeroBotRotationCommand());
                driverStick.Button(12).onTrue(drivetrain.stopAllCommand());

                driverStick.Button(5).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose1Red));
                // driverStick.Button(6).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose2Red));
                driverStick.Button(4).onTrue(drivetrain.goToPoseCommand(drivetrain.climbRightRed));
                driverStick.Button(3).onTrue(drivetrain.goToPoseCommand(drivetrain.climbLeftRed));
                driverStick.Button(2).onTrue(drivetrain.testTriangle());
                driverStick.Button(8).onTrue(drivetrain.trenchRightOutCommand());
                driverStick.Button(10).onTrue(drivetrain.trenchRightInCommand());
                driverStick.Button(1).onTrue(drivetrain.lookAtHubCommand());

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.b().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
                // -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // // Reset the field-centric heading on left bumper press.
                // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                // final var idle = new SwerveRequest.Idle();
                // return Commands.sequence(
                // // Reset our field centric heading to match the robot
                // // facing away from our alliance station wall (0 deg).
                // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // // Then slowly drive forward (away from us) for 5 seconds.
                // drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                // .withVelocityY(0)
                // .withRotationalRate(0))
                // .withTimeout(5.0),
                // // Finally idle for the rest of auton
                // drivetrain.applyRequest(() -> idle));
                return m_auto.getAutoCommand();
        }
}
