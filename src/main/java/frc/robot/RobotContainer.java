// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.SystemZero;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;
import frc.robot.controls.MayhemOperatorPad;
import frc.robot.controls.MayhemExtreme3dPro.Axis;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
        private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                         // second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final MayhemExtreme3dPro driverStick = new MayhemExtreme3dPro(0);
        // private final MayhemExtreme3dPro driverStick = null;
        private final MayhemOperatorPad operatorPad = new MayhemOperatorPad();
        private final MayhemLogitechAttack3 debugStick = new MayhemLogitechAttack3(2);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final AutoChooser m_auto = new AutoChooser();
        private static final Vision vision = new Vision();
        public static final CANBus canivore = new CANBus("canivore");

        // public static final TalonFX loaderMotor = new TalonFX(32, "canivore");
        // public static final TalonFX shooter1Motor = new TalonFX(33, "canivore");
        // public static final TalonFX hoodMotor = new TalonFX(31, "canivore");
        // public static final TalonFX intakeRollerMotor = new TalonFX(27, "canivore");
        // public static final TalonFX intakeArmMotor = new TalonFX(26, "canivore");
        // public static final TalonFX hopperMotor = new TalonFX(28, "canivore");
        // public static final TalonFX climberElevatorPivotMotor = new TalonFX(25,
        // "canivore");
        // public static final TalonFX climberElevatorMotor = new TalonFX(24,
        // "canivore");
        // public static final TalonFX climberLowerClawsMotor = new TalonFX(23,
        // "canivore");

        public static final TalonFX loaderMotor = new TalonFX(32, canivore);
        public static final TalonFX shooter1Motor = new TalonFX(33, canivore);
        public static final TalonFX hoodMotor = new TalonFX(31, canivore);
        public static final TalonFX intakeRollerMotor = new TalonFX(27, canivore);
        public static final TalonFX intakeArmMotor = new TalonFX(26, canivore);
        public static final TalonFX hopperMotor = new TalonFX(28, canivore);

        // public static final TalonFX climberElevatorPivotMotor = new TalonFX(25,
        // canivore);
        // public static final TalonFX climberElevatorMotor = new TalonFX(24, canivore);
        // public static final TalonFX climberLowerClawsMotor = new TalonFX(23,
        // canivore);
        public static final TalonFX climberElevatorPivotMotor = null;
        public static final TalonFX climberElevatorMotor = null;
        public static final TalonFX climberLowerClawsMotor = null;

        public static final IntakeArm intakeArm = new IntakeArm(intakeArmMotor);
        public static final IntakeRollers intakeRollers = new IntakeRollers(intakeRollerMotor);
        public static final ClimberElevator climberElevator = new ClimberElevator(climberElevatorMotor);
        public static final ClimberElevatorPivot climberElevatorPivot = new ClimberElevatorPivot(
                        climberElevatorPivotMotor);
        public static final ClimberShortArmPivot climberShortArmPivot = new ClimberShortArmPivot(
                        climberLowerClawsMotor);

        public static final GameTimer gameTimer = new GameTimer();
        public static final Shooter shooter = new Shooter(shooter1Motor);
        public static final Hopper hopper = new Hopper(hopperMotor);
        public static final ShooterHood shooterHood = new ShooterHood(hoodMotor);
        public static final Loader loader = new Loader(loaderMotor);
        public static final LaunchingTower launchingTower = new LaunchingTower(shooter, shooterHood, loader, hopper,
                        intakeRollers);

        public RobotContainer() {
                configureBindings();
                configureDebugBindings();
                configureNamedCommands();
                if (driverStick != null) {
                        configureDriverstickBindings();
                }

                m_auto.addAuto("Stand Still", new WaitCommand(2));
                m_auto.addAuto("Shoot Center Climb", new PathPlannerAuto("Shoot Center Climb"));
                m_auto.addAuto("Shoot Depot Shoot Left", new PathPlannerAuto("Shoot Depot Shoot Left"));
                m_auto.addAuto("Shoot Outpost Shoot Right", new PathPlannerAuto("Shoot Outpost Shoot Right"));
                m_auto.addAuto("Trench Left Shoot Twice", new PathPlannerAuto("Trench Left Shoot Twice"));
        }

        private void configureNamedCommands() {
                // NamedCommands.registerCommand("Shoot-8s", new SequentialCommandGroup(
                // new DrivePointToHub(),
                // launchingTower.fireFuelCommand().withTimeout(8)));
                NamedCommands.registerCommand("Shoot-8s", new ParallelRaceGroup(
                                new DrivePointToHub().repeatedly(),
                                launchingTower.fireFuelCommand().withTimeout(8)));
                NamedCommands.registerCommand("Shoot-4s", new ParallelRaceGroup(
                                new DrivePointToHub().repeatedly(),
                                launchingTower.fireFuelCommand().withTimeout(4)));
                NamedCommands.registerCommand("Intake Down",
                                new ParallelCommandGroup(intakeArm.goToDownCommand(), intakeRollers.intakeCommand()));
                NamedCommands.registerCommand("Intake Up",
                                new ParallelCommandGroup(intakeArm.goToUpCommand(), intakeRollers.turnOffCommand()));
                // NamedCommands.registerCommand("Outtake", endEffector.setSpeedCmd(-.8));
                // NamedCommands.registerCommand("IntakeStart", endEffector.setSpeedCmd(0.8));
                // NamedCommands.registerCommand("Algae High", ArmPositionAutoCmd(-750, -929,
                // -13));
                // NamedCommands.registerCommand("Algae High Remove",
                // ArmPositionAutoCmd(-485, -900, -13).withTimeout(2.0));

                // NamedCommands.registerCommand("IntakeStop", endEffector.setSpeedCmd(0.0));
                // NamedCommands.registerCommand("AutoHP", ArmPositionAutoCmd(-480, -1200, 0));
                // NamedCommands.registerCommand("TeleHP", ArmPositionCmd(-693, 1103, -219));
                // NamedCommands.registerCommand("Stow", ArmPositionAutoCmd(0, -1764, 0));

        }

        private void configureBindings() {
                // // Note that X is defined as forward according to WPILib convention,
                // // and Y is defined as to the left according to WPILib convention.
                // drivetrain.setDefaultCommand(
                // // Drivetrain will execute this command periodically
                // drivetrain.applyRequest(
                // () ->
                // // Drive forward with negative Y (forward)
                // drive.withVelocityX(-driverStick.getRawAxis(Axis.Y) * MaxSpeed)
                // // Drive left with negative X (left)
                // .withVelocityY(-driverStick.getRawAxis(Axis.X)
                // * MaxSpeed)
                // // Drive counter-clockwise with negative X (left)
                // .withRotationalRate(-driverStick.getRawAxis(Axis.Z)
                // * MaxAngularRate)));

                // driverStick.Button(6).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose2Red));
                // driverStick.Button(4).onTrue(drivetrain.goToPoseCommand(drivetrain.climbRightRed));
                // driverStick.Button(3).onTrue(drivetrain.goToPoseCommand(drivetrain.climbLeftRed));
                // driverStick.Button(2).whileTrue(
                // drivetrain.fireWhileDriving(driverStick.Axis(Axis.Y),
                // driverStick.Axis(Axis.X)));
                // driverStick.Button(9).whileTrue(drivetrain.strafeWhileFiringCommand());
                // driverStick.Button(7).whileTrue(
                // drivetrain.fireWhileDriving(driverStick.Axis(Axis.Y),
                // driverStick.Axis(Axis.X)));

                // driverStick.Button(1)
                // .whileTrue(new SequentialCommandGroup(new DrivePointToHub(),
                // drivetrain.lockWheels()));
                // driverStick.Button(5).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose1Red));
                // driverStick.Button(6).whileTrue(drivetrain.lockWheels());
                // driverStick.Button(8).onTrue(drivetrain.trenchRightOutCommand());
                // driverStick.Button(10).onTrue(drivetrain.trenchRightInCommand());
                // driverStick.Button(11).onTrue(drivetrain.zeroBotRotationCommand());
                // driverStick.Button(12).onTrue(drivetrain.stopAllCommand());

                // driverStick.Button(7).onTrue(drivetrain.bumpCommand());

                // climb down from L1
                // operatorPad.Button(2).onTrue(new ClimbDownFromLowerRung());
                // climb up to L1
                // operatorPad.Button(3).onTrue(new ClimbToLowerRung());
                // future climb to next rung
                // operatorPad.Button(4).onTrue(command);
                // intake on and off
                operatorPad.Button(5).onTrue(
                                new ParallelCommandGroup(intakeRollers.intakeCommand(), hopper.turnOnCommand()));
                operatorPad.Button(5)
                                .onFalse(new ParallelCommandGroup(intakeRollers.turnOffCommand(),
                                                hopper.turnOffCommand()));

                operatorPad.Button(7).onTrue(
                                new ParallelCommandGroup(intakeRollers.outtakeCommand(), hopper.reverseCommand()));
                operatorPad.Button(7)
                                .onFalse(new ParallelCommandGroup(intakeRollers.turnOffCommand(),
                                                hopper.turnOffCommand()));

                // intake in and out
                operatorPad.Button(6).onTrue(intakeArm.goToUpCommand());
                operatorPad.Button(8).onTrue(intakeArm.goToDownCommand());

                // zero all
                operatorPad.Button(10).onTrue(new SystemZero());

                // shoot sequence
                operatorPad.D_PAD_UP.onTrue(launchingTower.fireFuelCommand());
                operatorPad.D_PAD_UP.onFalse(new SequentialCommandGroup(
                                loader.turnOffCommand(),
                                hopper.turnOffCommand(),
                                shooter.setVelocityCommand(0),
                                shooterHood.SetPositiongByMMCommand(0)));

                // climber manual
                climberElevatorPivot.setDefaultCommand(
                                climberElevatorPivot
                                                .controlWithAxis(operatorPad.Axis(
                                                                frc.robot.controls.MayhemOperatorPad.Axis.RightX)));
                climberElevator.setDefaultCommand(
                                climberElevator.controlWithAxis(
                                                operatorPad.Axis(frc.robot.controls.MayhemOperatorPad.Axis.LeftY)));

                // // Idle while the robot is disabled. This ensures the configured
                // // neutral mode is applied to the drive motors while disabled.
                // final var idle = new SwerveRequest.Idle();
                // RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() ->
                // idle).ignoringDisable(true));

                // drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureDriverstickBindings() {
                // driverStick.Button(6).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose2Red));
                // driverStick.Button(4).onTrue(drivetrain.goToPoseCommand(drivetrain.climbRightRed));
                // driverStick.Button(3).onTrue(drivetrain.goToPoseCommand(drivetrain.climbLeftRed));
                driverStick.Button(2).whileTrue(
                                drivetrain.fireWhileDriving(driverStick.Axis(Axis.Y),
                                                driverStick.Axis(Axis.X)));
                // driverStick.Button(9).whileTrue(drivetrain.strafeWhileFiringCommand());
                // driverStick.Button(7).whileTrue(
                // drivetrain.fireWhileDriving(driverStick.Axis(Axis.Y),
                // driverStick.Axis(Axis.X)));

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () ->
                                                // Drive forward with negative Y (forward)
                                                drive.withVelocityX(-driverStick.getRawAxis(Axis.Y) * MaxSpeed)
                                                                // Drive left with negative X (left)
                                                                .withVelocityY(-driverStick.getRawAxis(Axis.X)
                                                                                * MaxSpeed)
                                                                // Drive counter-clockwise with negative X (left)
                                                                .withRotationalRate(-driverStick.getRawAxis(Axis.Z)
                                                                                * MaxAngularRate)));

                driverStick.Button(1)
                                .whileTrue(new SequentialCommandGroup(new DrivePointToHub(), drivetrain.lockWheels()));
                driverStick.Button(5).onTrue(drivetrain.goToPoseCommand(drivetrain.shooterPose1Red));
                driverStick.Button(6).whileTrue(drivetrain.lockWheels());
                driverStick.Button(8).onTrue(drivetrain.trenchRightOutCommand());
                driverStick.Button(10).onTrue(drivetrain.trenchRightInCommand());
                driverStick.Button(11).onTrue(drivetrain.zeroBotRotationCommand());
                driverStick.Button(12).onTrue(drivetrain.stopAllCommand());

                driverStick.Button(7).onTrue(drivetrain.bumpCommand());

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureDebugBindings() {
                // debugStick.Button(8).onTrue(intakeRollers.setSpeedCommand(0.1));
                // debugStick.Button(8).onFalse(intakeRollers.setSpeedCommand(0.0));
                // debugStick.Button(1).onTrue(intakeArm.setPowerCommand(0));
                // debugStick.Button(4).onTrue(intakeArm.setPowerCommand(.1));

                // debugStick.Button(1).onTrue(climberElevator.setPositionCommand(3.5));
                // debugStick.Button(4).onTrue(climberElevator.setPositionCommand(0));

                // climberElevator.setDefaultCommand(
                // climberElevator.controlWithAxis(debugStick.Axis(MayhemLogitechAttack3.Axis.Y)));
                // debugStick.Button(2).onTrue(climberElevator.zeroCommand());

                // operatorPad.Button(3).onTrue(shooter.setShooterSpeedCommand(.2));
                // operatorPad.Button(3).onFalse(shooter.setShooterSpeedCommand(0));

                // debugStick.Button(11).onTrue(shooter.offsetShooterVelocityCommand(5));
                // debugStick.Button(10).onTrue(shooter.offsetShooterVelocityCommand(-2.5));
                // operatorPad.Button(3).onTrue(shooter.setControlCommand(20));
                // operatorPad.Button(3).onFalse(shooter.setControlCommand(0));

                // shooterHood.setDefaultCommand(shooterHood
                // .controlWithAxis(operatorPad.Axis(frc.robot.controls.MayhemOperatorPad.Axis.LeftY)));
                // debugStick.Button(6).onTrue(shooterHood.offsetPositionCommand(1));
                // debugStick.Button(7).onTrue(shooterHood.offsetPositionCommand(-1));
                // debugStick.Button(8).onTrue()
                // operatorPad.Button(2).onTrue(hopper.turnOnCommand());
                // operatorPad.Button(2).onFalse(hopper.turnOffCommand());

                // climber elevator manual
                // climberElevator.controlWithAxis(operatorPad.Axis(MayhemOperatorPad.Axis.LeftY));

                // operatorPad.Button(5).onTrue(intakeRollers.intakeCommand());
                // operatorPad.Button(5).onFalse(intakeRollers.turnOffCommand());

                // operatorPad.Button(7).onTrue(intakeRollers.outtakeCommand());
                // operatorPad.Button(7).onFalse(intakeRollers.turnOffCommand());

                // operatorPad.Button(4).whileTrue(loader.setSpeedCommand(.2));

                // operatorPad.Button(2).onTrue(climberShortArmPivot.setPowerCommand(-.1));
                // operatorPad.Button(2).onFalse(climberShortArmPivot.setPowerCommand(0));
                // operatorPad.Button(3).onTrue(climberShortArmPivot.setPowerCommand(.1));
                // operatorPad.Button(3).onFalse(climberShortArmPivot.setPowerCommand(0));

                // operatorPad.Button(4).onTrue(loader.setSpeedCommand(.5));
                // operatorPad.Button(4).onFalse(loader.setSpeedCommand(0));
                // intakeArm.setDefaultCommand(intakeArm.controlWithAxis(operatorPad.Axis(frc.robot.controls.MayhemOperatorPad.Axis.RightY)));

                debugStick.Button(3).onTrue(shooter.offsetShooterVelocityCommand(2));
                debugStick.Button(2).onTrue(shooter.offsetShooterVelocityCommand(-2));

                debugStick.Button(4).onTrue(shooterHood.offsetPositionCommand(1));
                debugStick.Button(5).onTrue(shooterHood.offsetPositionCommand(-1));

                debugStick.Button(7).onTrue(loader.setSpeedCommand(.75));
                debugStick.Button(7).onFalse(loader.setSpeedCommand(0));

                debugStick.Button(6).onTrue(shooter.setVelocityCommand(0));

        }

        public Command getAutonomousCommand() {
                // return m_auto.getAutoCommand();
                return m_auto.getAutoCommand();
        }
}
