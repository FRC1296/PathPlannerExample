// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.AutonmousRoutineOne;
import frc.robot.autonomous.IAuto;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.drivesystem.CommandSwerveDrivetrain;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.led.LedSubsystem;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.WristSubsystem;

/**
 * This is our main robot class
 */
public class FMJRobot {
    // Boolean that identifies whether this instance is the Competition Robot or the Practice Robot
    private boolean competitionBot;

    // The current alliance for our robot
    private boolean currAllianceBlue;

    // Choose auton for robot to run
    private SendableChooser<Command> autonChooser = new SendableChooser<>();
    private Command autonOption1 = null;
    private Command autonOption2 = null;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_CONTROL); // My joystick
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_CONTROL);
    private final CommandXboxController characterizer = new CommandXboxController(Constants.SYSID_CONTROL);

    public CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                Constants.CTRESwerve.DrivetrainConstants, Constants.CTRESwerve.CompFrontLeftSwerveModule,
                Constants.CTRESwerve.CompFrontRightSwerveModule, Constants.CTRESwerve.CompBackLeftSwerveModule,
                Constants.CTRESwerve.CompBackRightSwerveModule);

    private final SwerveRequest.FieldCentric drive = 
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
            .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.RobotCentric trackDrive =
        new SwerveRequest.RobotCentric()
            .withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
            .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    /* Path follower */
    //private Command runAuto = drivetrain.getAutoPath("Tests");
    //private Command runAuto;
    private final Telemetry logger = new Telemetry(Constants.CTRESwerve.MAXSPEED);

    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private FeederSubsystem feeder = new FeederSubsystem();
    private DigitalInput beamBreak = new DigitalInput(Constants.DP_BEAMBREAK);
    private DigitalInput climberLimit = new DigitalInput(Constants.DP_CLIMBER_LIMIT);
    private DigitalInput wristLimit = new DigitalInput(Constants.DP_WRIST_LIMIT);
    private WristSubsystem wrist = new WristSubsystem();
    private ClimberSubsystem climber = new ClimberSubsystem();
    private LedSubsystem led = null;

    private BooleanPublisher autoAimPublisher;
    private BooleanPublisher beamBooleanPublisher;
    private BooleanPublisher climberLimitPublisher;
    private BooleanPublisher wristLimitPublisher;
    private BooleanPublisher noteRangePublisher;
    private BooleanPublisher tagRangePublisher;
    private DoublePublisher limelightTagPublisher;
    private DoublePublisher tagDistancePublisher;
    private DoublePublisher limelightNotePublisher;
    private DoublePublisher noteDistancePublisher;

    private BooleanSubscriber autoAimSubscriber;
    private BooleanSubscriber beamBreakSubscriber;
    private BooleanSubscriber climberLimitSubscriber;
    private BooleanSubscriber wristLimitSubscriber;
    private BooleanSubscriber tagRangeSubscriber;
    private BooleanSubscriber noteRangeSubscriber;
    private DoubleSubscriber tagControllerSub;
    private DoubleSubscriber noteControllerSub;

    private int speakerTagNumber;
    private int ampTagNumber;
    private List<Integer> stageTags;

    public static Orchestra FMJOrchestra = new Orchestra();

    public FMJRobot(boolean compBot, boolean isBlue) {
        competitionBot = compBot;
        currAllianceBlue = isBlue;

        if (currAllianceBlue) {
            speakerTagNumber = Constants.BlueAllianceAprilTags.SPEAKER_CENTERED;
            ampTagNumber = Constants.BlueAllianceAprilTags.AMP;
            stageTags = Arrays.asList(
                Constants.BlueAllianceAprilTags.STAGE_CENTER,
                Constants.BlueAllianceAprilTags.STAGE_LEFT,
                Constants.BlueAllianceAprilTags.STAGE_RIGHT
            );
        } else {
            speakerTagNumber = Constants.RedAllianceAprilTags.SPEAKER_CENTERED;
            ampTagNumber = Constants.RedAllianceAprilTags.AMP;
            stageTags = Arrays.asList(
                Constants.RedAllianceAprilTags.STAGE_CENTER,
                Constants.RedAllianceAprilTags.STAGE_LEFT,
                Constants.RedAllianceAprilTags.STAGE_RIGHT
            );
        }

        LedSubsystem led = new LedSubsystem();

        configureNetworkTable();

        drivetrain.configurePublishers();

        configureDriverController();

        configureOperatorController();

        // If control panel is available then configure
        if (DriverStation.isJoystickConnected(Constants.CUSTOM_CONTROL) == true) {
            configureCustomController();
        }

        // If third joystick is connected  then configure it for system characterization
        if (DriverStation.isJoystickConnected(Constants.SYSID_CONTROL) == true) {
            configureCharacterizationController();
        }

        drivetrain.registerTelemetry(logger::telemeterize);
        //drivetrain.configurePathPlanner(!currAllianceBlue);

        configureAutonomousOptions();

        configureShuffleboard();
    }

    private void configureDriverController() {
        Command defaultDrive = drivetrain.applyRequest(
                    () -> drive
                        .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                        .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                        .withRotationalRate(-driver.getRightX() * Constants.CTRESwerve.MAXANGULARRATE));

        drivetrain.setDefaultCommand(defaultDrive);

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.25 * Constants.CTRESwerve.MAXSPEED).withVelocityY(0)));
        driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.25 * Constants.CTRESwerve.MAXSPEED).withVelocityY(0)));

        // Create simulated trigger for beambreak
        Trigger haveNoteTrigger = new Trigger(beamBreakSubscriber);
        Trigger noNoteTrigger = haveNoteTrigger.negate();

        // Create simulated note range trigger
        Trigger noteRangeTrigger = new Trigger(noteRangeSubscriber);

        // Create auto aim trigger
        Trigger autoAimTrigger = new Trigger(autoAimSubscriber);

        // Create simulated note range trigger
        Trigger tagRangeTrigger = new Trigger(tagRangeSubscriber);

        // Create climber position reset trigger
        Trigger climberPositionReset = new Trigger(climberLimitSubscriber);
        // TODO: uncomment this to enable the climber position reset wtih limit switch
        climberPositionReset.onTrue(
            Commands.runOnce(climber::stopClimber, climber)
                .andThen(Commands.runOnce(climber::resetPosition, climber))
        );

        // Create wrist position reset trigger
        Trigger wristPositionReset = new Trigger(wristLimitSubscriber);

        Command noteDrive = drivetrain.applyRequest(
                    () -> trackDrive
                        .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                        .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                        .withRotationalRate(noteControllerSub.get() * Constants.CTRESwerve.MAXANGULARRATE));


        Command tagDrive = drivetrain.applyRequest(
                    () -> trackDrive
                        .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                        .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                        .withRotationalRate(tagControllerSub.get() * Constants.CTRESwerve.MAXANGULARRATE));

        driver.rightBumper().onTrue(new IntakeCommand(intake, feeder, wrist));

        // Command to run feeder to shoot
        // TODO: add logic to only allow if shooter is at speed
        driver.rightTrigger().and(haveNoteTrigger)
            .whileTrue(Commands.run(feeder::runFeeder, feeder))
            .whileFalse(Commands.run(feeder::stopFeeder, feeder));
    }

    private void configureOperatorController() {
        operator.leftTrigger().and(operator.back().negate())
            .whileTrue(Commands.run(intake::runIntake, intake))
            .whileFalse(Commands.run(intake::stopIntake, intake));

        operator.leftTrigger().and(operator.back())
            .whileTrue(Commands.run(intake::reverseIntake, intake))
            .whileFalse(Commands.run(intake::stopIntake, intake));

        //operator.leftTrigger()
        //    .whileTrue(Commands.run(shooter::runShooter, shooter))
        //    .whileFalse(Commands.run(shooter::stopShooter, shooter));
        operator.rightBumper().toggleOnTrue(Commands.startEnd(shooter::runShooter, shooter::stopShooter, shooter));
        operator.leftBumper().toggleOnTrue(Commands.startEnd(shooter::setAmpSpeed, shooter::stopShooter, shooter));

        operator.rightTrigger().and(operator.back().negate())
            .whileTrue(Commands.run(feeder::runFeeder, feeder))
            .whileFalse(Commands.run(feeder::stopFeeder, feeder));

        operator.rightTrigger().and(operator.back())
            .whileTrue(Commands.run(feeder::reverseFeeder, feeder))
            .whileFalse(Commands.run(feeder::stopFeeder, feeder));

        // Left Stick will run climber up or down
        operator.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
            .whileTrue(Commands.run(climber::climberDown, climber))
            .whileFalse(Commands.run(climber::stopClimber, climber));
        operator.axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
            .whileTrue(Commands.run(climber::climberUp, climber))
            .whileFalse(Commands.run(climber::stopClimber, climber));

        // POV will move Climber to set positions
        //operator.povUp().onTrue(Commands.run(climber::moveToTrapHeight, climber));

        operator.povUp().onTrue(Commands.run(climber::moveToAmpHeight, climber));

        operator.povDown().onTrue(Commands.run(climber::moveToZero, climber));

        // Right Stick will run Wrist up or down
        operator.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5)
            .whileTrue(Commands.run(wrist::wristDown, wrist))
            .whileFalse(Commands.run(wrist::stopWrist, wrist));
        operator.axisLessThan(XboxController.Axis.kRightY.value, -0.5)
            .whileTrue(Commands.run(wrist::wristUp, wrist))
            .whileFalse(Commands.run(wrist::stopWrist, wrist));

        // Bottons to move wrist to set positions
        operator.y().onTrue(Commands.run(wrist::moveToAmpHeight, wrist));

        operator.a().onTrue(Commands.run(wrist::moveToZero, wrist));

        // Turn off auto aim for targeting
        // TODO: validate that this works properly
        operator.start().toggleOnTrue(
            Commands.startEnd(
                () -> {
                    autoAimPublisher.set(false);
                },
                () -> {
                    autoAimPublisher.set(true);
                }
            )
        );

        // Turn off Stator Current Limit - WARNING - only to be done during climb or other extreme circumstances
        operator.x().and(operator.b()).onTrue(Commands.runOnce(climber::setToClimbMode, climber));
    }

    private void configureCustomController() {
    }

    // TODO: test sysid of swerve
    private void configureCharacterizationController() {
        //SignalLogger.enableAutoLogging(true);
        //SignalLogger.start();
        /* Bindings for drivetrain characterization */
        /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
        /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
        characterizer.back().and(characterizer.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        characterizer.back().and(characterizer.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        characterizer.start().and(characterizer.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        characterizer.start().and(characterizer.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Setup Shooter bindings
        characterizer.start().and(characterizer.a())
                .whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        characterizer.start().and(characterizer.b())
                .whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        characterizer.back().and(characterizer.a())
            .whileTrue(shooter.sysIdDynamic(Direction.kForward));
        characterizer.back().and(characterizer.b())
            .whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    }

    private void configureNetworkTable(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);
        
        BooleanTopic autoAimTopic = fmjTable.getBooleanTopic(Constants.NT_AUTO_AIM);
        autoAimPublisher = autoAimTopic.publish();
        autoAimPublisher.setDefault(true);
        autoAimSubscriber = autoAimTopic.subscribe(true);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        beamBooleanPublisher = beamTopic.publish();
        beamBooleanPublisher.setDefault(false);
        beamBreakSubscriber = beamTopic.subscribe(false);

        BooleanTopic climberTopic = fmjTable.getBooleanTopic(Constants.NT_CLIMBER_LIMIT);
        climberLimitPublisher = climberTopic.publish();
        climberLimitPublisher.setDefault(false);
        climberLimitSubscriber = climberTopic.subscribe(false);

        BooleanTopic wristTopic = fmjTable.getBooleanTopic(Constants.NT_WRIST_LIMIT);
        wristLimitPublisher = wristTopic.publish();
        wristLimitPublisher.setDefault(false);
        wristLimitSubscriber = wristTopic.subscribe(false);

        DoubleTopic tagTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_PID_VALUE);
        limelightTagPublisher = tagTopic.publish();
        limelightTagPublisher.setDefault(0);
        tagControllerSub = tagTopic.subscribe(0);

        BooleanTopic tagRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_TAG_RANGE);
        tagRangePublisher = tagRangeTopic.publish();
        tagRangePublisher.setDefault(false);
        tagRangeSubscriber = tagRangeTopic.subscribe(false);

        DoubleTopic tagDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_DISTANCE);
        tagDistancePublisher = tagDistanceTopic.publish();
        tagDistancePublisher.setDefault(0);

        DoubleTopic noteTopic = fmjTable.getDoubleTopic(Constants.NT_NOTE_LL_PID_VALUE);
        limelightNotePublisher = noteTopic.publish();
        limelightNotePublisher.setDefault(0);
        noteControllerSub = noteTopic.subscribe(0);

        BooleanTopic noteRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_NOTE_RANGE);
        noteRangePublisher = noteRangeTopic.publish();
        noteRangePublisher.setDefault(false);
        noteRangeSubscriber = noteRangeTopic.subscribe(false);

        DoubleTopic noteDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_NOTE_DISTANCE);
        noteDistancePublisher = noteDistanceTopic.publish();
        noteDistancePublisher.setDefault(0.0);
    }

    private void configureAutonomousOptions() {
        // Add Autonomous Options to Dashboard
        ShuffleboardTab robotTab = Shuffleboard.getTab("FMJ Robot");
        SendableRegistry.setName(autonChooser, "Autonomous Options");
        boolean redPath = true;
        boolean bluePath = false;

        // Create Option 1 and add to list of choices
        autonOption1 = new AutonmousRoutineOne(this, 0, 0, bluePath);
        autonChooser.addOption("Auton 1 Blue", autonOption1);

        autonOption2 = new AutonmousRoutineOne(this, 0, 0, redPath);
        autonChooser.addOption("Auton 1 Red", autonOption2);

        ShuffleboardLayout autonLayout = robotTab.getLayout("Autonomous", BuiltInLayouts.kList).withSize(2, 1).withPosition(0, 0);
        autonLayout.add(autonChooser).withSize(4, 1).withPosition(0, 0);
    }

    private void configureShuffleboard() {
        ShuffleboardTab robotTab = Shuffleboard.getTab("FMJ Robot");
    }

    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected();

        Pose2d initialPose;
        if (auton instanceof IAuto) {
            initialPose = ((IAuto)auton).getInitialPose();
        } else {
            initialPose = new Pose2d();
        }
        drivetrain.initializePoseForAutonomous(initialPose);

        return auton;
    }

    public void robotPeriodic() {
        // System.out.println("April Tag(tx) - '" + limelightTarget.getLimelightX() + "'");
        beamBooleanPublisher.set(!beamBreak.get());
        climberLimitPublisher.set(!climberLimit.get());
        wristLimitPublisher.set(!wristLimit.get());
    }

    public void autonomousInit() {
        autoAimPublisher.set(false);
    }

    public void autonomousPeriodic() {
    }

    public void teleopInit() {
        autoAimPublisher.set(false);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance.equals(Alliance.Blue)) {
            drivetrain.seedFieldRelative(new Pose2d(), 0.0);
        } else {
            drivetrain.seedFieldRelative(new Pose2d(), 180.0);
        }
    }

    public void teleopPeriodic() {
    }

    public ClimberSubsystem getClimberSubsystem() {
        return this.climber;
    }

    public CommandSwerveDrivetrain getDriveSubsystem() {
        return this.drivetrain;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intake;
    }

    public FeederSubsystem getFeederSubsystem() {
        return this.feeder;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return this.shooter;
    }

    public WristSubsystem getWristSubsystem() {
        return wrist;
    }

    public int getAmpTagNumber() {
        return ampTagNumber;
    }

    public int getSpeakerTagNumber() {
        return speakerTagNumber;
    }
}
