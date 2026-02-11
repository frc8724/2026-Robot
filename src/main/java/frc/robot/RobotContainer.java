// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.ClimbDownFromLowerRung;
import frc.robot.commands.ClimbToLowerRung;
import frc.robot.commands.SystemZero;
import frc.robot.controls.JoystickPOVButton;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;
import frc.robot.controls.MayhemOperatorPad;
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

        private final MayhemExtreme3dPro driverStick = new MayhemExtreme3dPro(1);
        private final MayhemOperatorPad operatorPad = new MayhemOperatorPad();

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final AutoChooser m_auto = new AutoChooser();
        private static final Vision vision = new Vision();
        public static final TalonFX motor20 = new TalonFX(20);
        public static final IntakeArm intakeArm = new IntakeArm(null);
        public static final IntakeRollers intakeRollers = new IntakeRollers(null);
        public static final ClimberElevator climberElevator = new ClimberElevator(motor20);
        public static final ClimberElevatorPivot climberElevatorPivot = new ClimberElevatorPivot(null);
        public static final ClimberShortArmPivot climberShortArmPivot = new ClimberShortArmPivot(null);

        public static final Shooter shooter = new Shooter(null);
        public static final Hopper hopper = new Hopper(null);
        public static final ShooterHood shooterHood = new ShooterHood(null);
        public static final Loader loader = new Loader(null);
        public static final LaunchingTower launchingTower = new LaunchingTower(shooter, shooterHood, loader, hopper);

        public RobotContainer() {

                configureBindings();

                m_auto.addAuto("Shoot Center Climb", new PathPlannerAuto("Shoot Center Climb"));
                m_auto.addAuto("Shoot Depot Shoot Left", new PathPlannerAuto("Shoot Depot Shoot Left"));
                m_auto.addAuto("Shoot Outpost Shoot Right", new PathPlannerAuto("Shoot Outpost Shoot Right"));

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
                driverStick.Button(9).whileTrue(drivetrain.strafeWhileFiringCommand());

                // operatorPad.Button(8).onTrue(intakeRollers.setSpeedCommand(0.1));
                // operatorPad.Button(8).onFalse(intakeRollers.setSpeedCommand(0.0));
                // operatorPad.Button(1).onTrue(intakeArm.setPowerCommand(0));
                // operatorPad.Button(4).onTrue(intakeArm.setPowerCommand(.1));

                // operatorPad.Button(1).onTrue(climberElevator.setPositionCommand(3.5));
                // operatorPad.Button(4).onTrue(climberElevator.setPositionCommand(0));

                // climberElevator.setDefaultCommand(
                // climberElevator.controlWithAxis(operatorPad.Axis(MayhemLogitechAttack3.Axis.Y)));
                // operatorPad.Button(2).onTrue(climberElevator.zeroCommand());

                // zero all
                operatorPad.Button(10).onTrue(new SystemZero());
                // intake on and off
                operatorPad.Button(5).onTrue(intakeRollers.turnOnCommand());
                operatorPad.Button(5).onFalse(intakeRollers.turnOffCommand());
                // intake in and out
                operatorPad.Button(6).onTrue(intakeArm.goToUpCommand());
                operatorPad.Button(8).onTrue(intakeArm.goToDownCommand());
                // shoot sequence
                operatorPad.D_PAD_UP.onTrue(launchingTower.fireFuelCommand());
                // climb up to L1
                operatorPad.Button(3).onTrue(new ClimbToLowerRung());
                // climb down from L1
                operatorPad.Button(2).onTrue(new ClimbDownFromLowerRung());
                // future climb to next rung
                // operatorPad.Button(4).onTrue(command);
                // climber elevator manual
                climberElevator.controlWithAxis(operatorPad.Axis(MayhemOperatorPad.Axis.Y));

                driverStick.Button(6).whileTrue(drivetrain.lockWheels());
                driverStick.Button(1)
                                .whileTrue(new SequentialCommandGroup(new DrivePointToHub(), drivetrain.lockWheels()));

                driverStick.PovButton(JoystickPOVButton.NORTH).onTrue(drivetrain.bumpCommand());

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
                // return m_auto.getAutoCommand();
                return new SequentialCommandGroup(
                                new SystemZero(),
                                m_auto.getAutoCommand());
        }
}
