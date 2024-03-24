package frc.robot.drivesystem;

import static edu.wpi.first.units.Units.Volts;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private StringPublisher gyroInitializationStatus;

    private DoublePublisher yawPublisher;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("swervetransstate", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this,
                    "SwerveTranslation"));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("swerverotatestate", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this,
                    "SwerveRotation"));

    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("swervesteerstate", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this,
                    "SwerveSteer"));

    /* Change this to the sysid routine you want to test - uncomment the one to use*/
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;
    //private final SysIdRoutine RoutineToApply = SysIdRoutineRotation;
    //private final SysIdRoutine RoutineToApply = SysIdRoutineSteer;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        //configurePathPlanner(true);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        //configurePathPlanner(true);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void configurePublishers() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);
        StringTopic gyroStatusTopic = fmjTable.getStringTopic("gyroInitialization");
        gyroInitializationStatus = gyroStatusTopic.publish();
        gyroInitializationStatus.setDefault("Uninitialized.");

        DoubleTopic yawTopic = fmjTable.getDoubleTopic(Constants.NT_YAW);
        yawPublisher = yawTopic.publish();
        yawPublisher.setDefault(0.0);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(PathPlannerPath path) {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        return new FollowPathHolonomic(
                path,
                ()->this.getState().Pose, // Robot pose supplier
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(10.0, 0.0, 0.0), // Rotation PID constants
                        Constants.CTRESwerve.kSpeedAt12VoltsMps, // Max module speed, in m/s
                        driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {return false;},
                this // Reference to this subsystem to set requirements
        );
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Initizliaze the Pigeon and Odometry position
     * @param pose Pose2d with x,y coordinates and directory(degree) robot is current positioned at
     * @return true for success, false for failure
     */
    public boolean initializePoseForAutonomous(Pose2d pose) {
        boolean ret = true;
        StatusCode result = this.getPigeon2().setYaw(pose.getRotation().getDegrees());
        if (!result.isOK()) {
            ret = false;
            System.err.println("----------------------------------------");
            System.err.println("---------GYRO INITIALIZATION FAILED - " + result.getName() + " - " + result.getDescription());
            System.err.println("----------------------------------------");
        }
        result = this.getPigeon2().setYaw(pose.getRotation().getDegrees());

        this.seedFieldRelative(pose, pose.getRotation().getDegrees());

        return ret;
    }


    /**
     * Copied from SwerveDrivetrain.seedFieldRelative(Pose2d location)
     * Added a yaw element to use a direction rather than the current yaw
     * 
     * @param location
     * @param yaw
     */
    public void seedFieldRelative(Pose2d location, double yaw) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(Rotation2d.fromDegrees(yaw), m_modulePositions, location);
            /* We need to update our cached pose immediately so that race conditions don't happen */
            m_cachedState.Pose = location;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void periodic() {
        yawPublisher.set(m_yawGetter.getValue());
    }

    
}
