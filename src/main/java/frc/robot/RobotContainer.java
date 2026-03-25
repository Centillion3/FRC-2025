// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralManipulator.CoralManipulatorPivotState;
import frc.robot.Constants.CoralManipulator.CoralManipulatorRollerState;
import frc.robot.commands.AutoAlgaeRemoval;
import frc.robot.commands.AutoAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Deepclimb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final CommandPS4Controller ps4Controller = new CommandPS4Controller(2);

    private final Trigger autoAlignLeftTrigger = driveController.leftBumper();
    private final Trigger autoAlignRightTrigger = driveController.rightBumper();
    private final Trigger autoAlgaeRemovalTrigger = driveController.rightTrigger(0.1);

    private final Trigger manualAlignLeft = driveController.povLeft();
    private final Trigger manualAlignDown = driveController.povDown();
    private final Trigger manualAlignRight = driveController.povRight();
    private final Trigger manualAlignUp = driveController.povUp();

    private final Trigger forceVisionTrigger = driveController.x();
    private final Trigger zeroTrigger = driveController.a();
    private final Trigger brakeTrigger = driveController.b();

    private final Trigger outtakeTrigger = secondaryController.rightTrigger(0.1);
    private final Trigger intakeTrigger = secondaryController.rightBumper();
    private final Trigger outtakeTroughTrigger = secondaryController.povUp();
    private final Trigger AlgaeRemovalTrigger = secondaryController.a();
    private final Trigger l1Trigger = secondaryController.povLeft();
    private final Trigger l2Trigger = secondaryController.x();
    private final Trigger l3Trigger = secondaryController.y();
    private final Trigger l4Trigger = secondaryController.b();

    private final Trigger intakeAlgaeTrigger = secondaryController.leftBumper();
    private final Trigger outtakeAlgaeTrigger = secondaryController.leftTrigger(0.1);

    private final Trigger zeroElevatorTrigger = secondaryController.povDown();

    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final Elevator elevator = new Elevator();
    private final CoralManipulator coralManipulator = new CoralManipulator();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Deepclimb deepclimb = new Deepclimb();
    private final LED led = new LED();
    public final Vision vision = new Vision(drivetrain);

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;

    private final Trigger beamBreakLEDTrigger = new Trigger(coralManipulator :: hasCoral);

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, elevator, coralManipulator, led);

        drivetrain.setElevator(elevator);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.new DriveXbox(driveController)
        );

        autoAlignLeftTrigger.whileTrue(new AutoAlign(drivetrain, elevator, led, true));
        autoAlignRightTrigger.whileTrue(new AutoAlign(drivetrain, elevator, led, false));
        autoAlgaeRemovalTrigger.whileTrue(new AutoAlgaeRemoval(drivetrain, elevator, coralManipulator, led));

        manualAlignLeft.whileTrue(drivetrain.new ManualAlign(0, 0.15));
        manualAlignDown.whileTrue(drivetrain.new ManualAlign(-0.15, 0));
        manualAlignRight.whileTrue(drivetrain.new ManualAlign(0, -0.15));
        manualAlignUp.whileTrue(drivetrain.new ManualAlign(0.15, 0));

        forceVisionTrigger.whileTrue(new InstantCommand(() -> vision.forceVision()));
        zeroTrigger.onTrue(new InstantCommand(() -> drivetrain.zero()));
        brakeTrigger.whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);

        outtakeTrigger.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKE)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));
        outtakeTroughTrigger.onTrue(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.OUTTAKETROUGH)).onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));

        intakeTrigger.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.rainbow()),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_INTAKE),
            coralManipulator.new ChangeState(CoralManipulatorPivotState.INTAKE, CoralManipulatorRollerState.INTAKE)))
            .onFalse(coralManipulator.new ChangeRollerState(CoralManipulatorRollerState.PASSIVE));

        beamBreakLEDTrigger.onTrue(led.new BlinkLED(Color.kGreen, 0.25, 1, true));

        l1Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_L1),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L1)));

        l2Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kGreen)),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_L2),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L2)));

        l3Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kBlue)),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_L3),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L3)));

        l4Trigger.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> led.light(Color.kPurple)),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_L4),
            coralManipulator.new ChangePivotState(CoralManipulatorPivotState.L4)));

        AlgaeRemovalTrigger.onTrue(new ParallelCommandGroup(
            coralManipulator.new ChangeState(CoralManipulatorPivotState.L2, CoralManipulatorRollerState.OUTTAKE),
            elevator.new ChangeState(Elevator.ElevatorEvent.GO_L2)))
            .onFalse(new ParallelCommandGroup(
                coralManipulator.new ChangeState(CoralManipulatorPivotState.ALGAEREMOVAL, CoralManipulatorRollerState.OUTTAKE),
                elevator.new ChangeState(Elevator.ElevatorEvent.GO_L3)));

        intakeAlgaeTrigger.whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.INTAKE)));

        outtakeAlgaeTrigger.whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> led.light(Color.kTeal)),
                algaeIntake.new ChangeState(Constants.AlgaeIntake.AlgaeIntakeState.OUTTAKE, true)));

        zeroElevatorTrigger.onTrue(new InstantCommand(() -> elevator.toggleZeroElevator()))
            .onFalse(new InstantCommand(() -> elevator.toggleZeroElevator()));
    }

    public Command getAutonomousCommand() {
        drivetrain.resetPose(new Pose2d());
        return autoRoutines.getGeneratedRoutine().cmd();
    }
}