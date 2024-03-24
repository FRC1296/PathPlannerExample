package frc.robot;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

public final class Constants {
    /***********************************************************************/
    /* Robot Constants */
    public static final String ROBOT_NAME = "FMJ1296";
    public static final String ROBOT_NICKNAME = "Riff";
    public static final String ROBOT_VERSION = "v2024.1.0";
    public static final int    DRIVER_CONTROL = 0;
    public static final int    OPERATOR_CONTROL = 1;
    public static final int    CUSTOM_CONTROL = 2;
    public static final int    SYSID_CONTROL = 3;
    /***********************************************************************/

    /***********************************************************************/
    /* Network Table Constants */
    public static final String NETWORK_TABLE = "FMJRobot";
    public static final String NT_SPEED_CONTROL = "rawSpeedControlValue";
    public static final String NT_ANGLE_VELOCITY_CONTROL = "rawAngleVelocityControlValue";
    public static final String NT_HAVE_NOTE = "Have Note";
    public static final String NT_LIMELIGHT_NOTE_TABLE = "limelight-rhsnote";
    public static final String NT_LIMELIGHT_TAG_TABLE = "limelight-rhstag";
    public static final String NT_LL_VALID_TARGET = "tv";
    public static final String NT_LL_HZ_OFFSET = "tx";
    public static final String NT_LL_VT_OFFSET = "ty";
    public static final String NT_LL_TARGET_AREA = "ta";
    public static final String NT_LL_TARGET_SLEW = "ts";
    public static final String NT_AUTO_AIM = "autoAim";
    public static final String NT_INTAKE_BEAM_BROKEN = "intakeBB";
    public static final String NT_TAG_LL_PID_VALUE = "limelightTag";
    public static final String NT_TAG_LL_TAG_RANGE = "inAprilTagRange";
    public static final String NT_TAG_LL_DISTANCE = "aprilTagDistance";
    public static final String NT_NOTE_LL_PID_VALUE = "limelightNote";
    public static final String NT_TAG_LL_NOTE_RANGE = "inNoteRange";
    public static final String NT_TAG_LL_NOTE_DISTANCE = "noteDistance";
    public static final String NT_CLIMBER_LIMIT = "ClimberLimit";
    public static final String NT_WRIST_LIMIT = "WristLimit";
    public static final String NT_YAW = "yaw";
    public static final String NT_INTAKE_HAS_NOTE = "intakeHasNote";
    /***********************************************************************/

    /***********************************************************************/
    /* Limelight Distance Calculation Constants */
    // TODO : update to real values once determined
    public static final double LL_FRONT_MOUNT_ANGLE = 34; // The angle of the limelight camera (0 is horizontal, 90 is fully virtical)
    public static final double LL_FRONT_MOUNT_HEIGHT = 23.5; // The height in inches of the camera from ground
    public static final double LL_APRIL_TAG_HEIGHT = 58.25; // The height in inches of the april tag targets from ground
    public static final double LL_REAR_MOUNT_ANGLE = -40; // The angle of the limelight camera (0 is horizontal, 90 is fully virtical)
    public static final double LL_REAR_MOUNT_HEIGHT = 18.5; // The hight in inches of the camera from ground
    public static final double LL_NOTE_HEIGHT = 1.0; // The height in inches of the note off the ground
    public static final double LL_NOTE_KP = 0.02;
    public static final double LL_TAG_KP = 0.0325;
    public static final double LL_MIN_RANGE_NOTE = 1.0;
    public static final double LL_MIN_RANGE_TAG = 0.5;
    /***********************************************************************/

    //PWM Constants
    public static final int PWM_LED1 = 0;

    /** CAN Bus Configuration */
    public static final int CAN_PIGEON = 0;

    public static final int CAN_INTAKE = 30;
    public static final int CAN_CLIMBERLEFT = 50;
    public static final int CAN_CLIMBERRIGHT = 52;
    public static final int CAN_UPPERSHOOTER = 51;
    public static final int CAN_LOWERSHOOTER = 34;
    public static final int CAN_WRIST = 7;
    public static final int CAN_FEEDER = 5;

    public static final int DP_BEAMBREAK = 0;
    public static final int DP_CLIMBER_LIMIT = 1;
    public static final int DP_WRIST_LIMIT = 2;
    public static final int DP_WRIST_ENCODER_1 = 3;

    public static final AudioConfigs ORCHESTRA_CONFIGS = new AudioConfigs().withAllowMusicDurDisable(true);
    
    public static final class CTRESwerve {
        // 6 meters per second desired top speed
        public static final double MAXSPEED = 5;

        // 3/4 of a rotation per second max angular velocity
        public static final double MAXANGULARRATE = 2 * Math.PI;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 300.0;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.21;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 12.8;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kSteerMotorReversed = false;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "";

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        public static final SwerveDrivetrainConstants DrivetrainConstants =
                new SwerveDrivetrainConstants()
                    .withPigeon2Id(CAN_PIGEON)
                    .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator =
                new SwerveModuleConstantsFactory().withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches).withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains).withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps).withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);


        // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 21;
    private static final double kFrontLeftEncoderOffset = 0.06884765625;

    private static final double kFrontLeftXPosInches = 10.75;
    private static final double kFrontLeftYPosInches = 10.75;

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 13;
    private static final int kFrontRightEncoderId = 23;
    private static final double kFrontRightEncoderOffset = 0.4970703125;

    private static final double kFrontRightXPosInches = 10.75;
    private static final double kFrontRightYPosInches = -10.75;

    // Back Left
    private static final int kBackLeftDriveMotorId = 2;
    private static final int kBackLeftSteerMotorId = 12;
    private static final int kBackLeftEncoderId = 22;
    private static final double kBackLeftEncoderOffset = 0.144775390625;

    private static final double kBackLeftXPosInches = -10.75;
    private static final double kBackLeftYPosInches = 10.75;

    // Back Right
    private static final int kBackRightDriveMotorId = 4;
    private static final int kBackRightSteerMotorId = 14;
    private static final int kBackRightEncoderId = 24;
    private static final double kBackRightEncoderOffset = 0.022216796875;

    private static final double kBackRightXPosInches = -10.75;
    private static final double kBackRightYPosInches = -10.75;


        // Competition Bot Swerve Module Configurations
        public static final SwerveModuleConstants CompFrontLeftSwerveModule =
                ConstantCreator.createModuleConstants(kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches),
                        Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants CompFrontRightSwerveModule =
                ConstantCreator.createModuleConstants(kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                        Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
        public static final SwerveModuleConstants CompBackLeftSwerveModule = 
                ConstantCreator.createModuleConstants(kBackLeftSteerMotorId, 
                        kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, 
                        Units.inchesToMeters(kBackLeftXPosInches),
                        Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
        public static final SwerveModuleConstants CompBackRightSwerveModule =
                ConstantCreator.createModuleConstants(kBackRightSteerMotorId,
                        kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches),
                        Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
    }

    public static final class BlueAllianceAprilTags {
        public static final int AMP = 6;
        public static final int SPEAKER_CENTERED = 7;
        public static final int SPEAKER_OFFSET = 8;
        public static final int SOURCE_CENTERED = 9;
        public static final int SOURCE_OFFSET = 10;
        public static final int STAGE_CENTER = 14;
        public static final int STAGE_LEFT = 15;
        public static final int STAGE_RIGHT = 16;
    }

    public static final class RedAllianceAprilTags {
        public static final int AMP = 5;
        public static final int SPEAKER_CENTERED = 4;
        public static final int SPEAKER_OFFSET = 3;
        public static final int SOURCE_CENTERED = 1;
        public static final int SOURCE_OFFSET = 2;
        public static final int STAGE_CENTER = 13;
        public static final int STAGE_LEFT = 11;
        public static final int STAGE_RIGHT = 12;
    }
}
