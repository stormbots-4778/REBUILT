package frc.robot.configuration;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotConfiguration {
    // all in meters
    public static final class ChassisConfig {
        // Distance between centers of my right and left wheels
        public static final double trackWidth = Units.inchesToMeters(26.5);
        // Distance between my front and back wheels
        public static final double wheelBase = Units.inchesToMeters(26.5);

        public static final double wheelDiameter = 0.074;
        public static final double wheelCircumferenceMeters = wheelDiameter * Math.PI;
    }

    public static final class DriveConfig {
        public static final double maxTeleopSpeed = 6;
        public static final double maxDrivetrainSpeed = 7;
        public static final double maxSpeedLimited = 0.25; // while shooting
        public static final double maxAngularSpeed = Math.PI * 4;

        private static final int drivingMotorPinionTeeth = 15;
        public static final double drivingMotorReduction = (45.0 * 20) / (drivingMotorPinionTeeth * 15);

        public static final double drivingMotorFreeSpeedRps = 6784 / 60;
        public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps
                * ChassisConfig.wheelCircumferenceMeters)
                / drivingMotorReduction;

        public static final int pigeonCAN = 32;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(ChassisConfig.wheelBase / 2, ChassisConfig.trackWidth / 2),
                new Translation2d(ChassisConfig.wheelBase / 2, -ChassisConfig.trackWidth / 2),
                new Translation2d(-ChassisConfig.wheelBase / 2, ChassisConfig.trackWidth / 2),
                new Translation2d(-ChassisConfig.wheelBase / 2, -ChassisConfig.trackWidth / 2));

        public static final class ModuleConfigs {
            // Angular offsets of the modules relative to the chassis in radians
            public static final double frontLeftCAO = -Math.PI / 2;
            public static final double frontRightCAO = 0;
            public static final double backLeftCAO = Math.PI;
            public static final double backRightCAO = Math.PI / 2;

            // CANs
            public static final int frontLeftDriveCAN = 1;
            public static final int frontRightDriveCAN = 3;
            public static final int backLeftDriveCAN = 7;
            public static final int backRightDriveCAN = 5;

            public static final int frontLeftTurnCAN = 10;
            public static final int frontRightTurnCAN = 14;
            public static final int backLeftTurnCAN = 6;
            public static final int backRightTurnCAN = 8;
        }

        public static final class MAXSwerveModule {
            public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
            public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
            public static final SparkMaxConfig turningConfigFR = new SparkMaxConfig(); // front right motor is tight,
                                                                                       // use separate pid

            static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double drivingFactor = ChassisConfig.wheelDiameter * Math.PI
                        / drivingMotorReduction;
                double turningFactor = 2 * Math.PI;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50); // 40 to david, 50 mura
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.02, 0, 0)
                        .outputRange(-1, 1).feedForward.kV(1.98).kA(0.25);

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);

                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0) // radians per second
                        // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for
                        // V1):
                        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);

                turningConfigFR
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);

                turningConfigFR.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0) // radians per second
                        // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for
                        // V1):
                        .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

                turningConfigFR.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(4, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
            }
        }
    }

    public static final class FeederConfig {
        public static final double feederSpeedSlow = -1000;
        public static final double feederSpeed = 3500;
        public static final int feedMotorLeftCAN = 30;
        public static final SparkMaxConfig feedMotorLeftConfig = new SparkMaxConfig();
        public static final int feedMotorRightCAN = 31;
        public static final SparkMaxConfig feedMotorRightConfig = new SparkMaxConfig();

        public static final int conveyorCAN = 17;
        public static final double conveyorSpeed = 1500;
        public static final double conveyorSpeedShoot = 3000;
        public static final SparkMaxConfig conveyorConfig = new SparkMaxConfig();

        static {
            conveyorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.000005, 0.0000001, 0);
            conveyorConfig.closedLoop.feedForward.kV(0.0021);
            conveyorConfig.smartCurrentLimit(45);
            conveyorConfig.closedLoop.maxMotion.maxAcceleration(15000);
            conveyorConfig.closedLoop.maxMotion.cruiseVelocity(5000);
            conveyorConfig.inverted(true);

            feedMotorLeftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.00003, 0, 0.0005)
                    .iZone(200);
            feedMotorLeftConfig.closedLoop.feedForward.kV(0.002225);
            feedMotorLeftConfig.smartCurrentLimit(60);

            feedMotorRightConfig.follow(feedMotorLeftCAN, true);
        }
    }

    public static final class ShooterConfig {
        public static final double SHOOTER_IDLE_SPEED = 1000;

        public static final int shooterLeft1CAN = 40;
        public static final SparkMaxConfig shooterLeft1Config = new SparkMaxConfig();
        public static final int shooterLeft2CAN = 41;
        public static final SparkMaxConfig shooterLeft2Config = new SparkMaxConfig();
        public static final int shooterRight1CAN = 50;
        public static final SparkMaxConfig shooterRight1Config = new SparkMaxConfig();
        public static final int shooterRight2CAN = 51;
        public static final SparkMaxConfig shooterRight2Config = new SparkMaxConfig();

        private static final int shooterCurrentLimit = 60;

        public static final int hoodLeftCAN = 20;
        public static final SparkMaxConfig hoodLeftConfig = new SparkMaxConfig();
        public static final int hoodRightCAN = 21;
        public static final SparkMaxConfig hoodRightConfig = new SparkMaxConfig();

        static {
            shooterLeft1Config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.0001, 0.00000001, 0.000025)
                    .iZone(100)
                    .iMaxAccum(0.2);
            shooterLeft1Config.closedLoop.feedForward.kV(0.0025);
            shooterLeft1Config.smartCurrentLimit(shooterCurrentLimit);
            shooterLeft1Config.closedLoop.maxMotion.maxAcceleration(20_000);
            shooterLeft1Config.closedLoop.maxMotion.cruiseVelocity(8000);
            shooterLeft1Config.inverted(true);
            shooterLeft1Config.encoder
                    .uvwAverageDepth(1)
                    .uvwMeasurementPeriod(8);

            shooterLeft2Config.apply(shooterLeft1Config);

            shooterRight1Config.apply(shooterLeft1Config);
            shooterRight1Config.inverted(false); // left is inverted, reverse here
            shooterRight2Config.apply(shooterRight1Config);

            hoodLeftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.1, 0.0005, 0)
                    .iZone(0.25);
            hoodLeftConfig.closedLoop.feedForward.kG(0.17);
            hoodLeftConfig.smartCurrentLimit(10);
            hoodRightConfig.follow(hoodLeftCAN, true);
        }
    }

    public static final class IntakeConfig {
        public static final int intakerCAN = 15;
        public static final int pivotCAN = 16;
        public final static double INTAKER_SPEED = 6000;

        public static final SparkMaxConfig intakerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

        static {
            intakerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.000002, 0, 0);
            intakerConfig.closedLoop.feedForward.kV(0.002);
            intakerConfig.smartCurrentLimit(35);

            pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.06, 0, 0);
            pivotConfig.inverted(true);
            pivotConfig.smartCurrentLimit(26);
            pivotConfig.idleMode(IdleMode.kCoast);
        }
    }

    public static final class VisionConfig {
        public static final String ll4Name = "limelight-sigma";
        public static final String ll2pName = "limelight-silly";
    }

    public static final class LightsConfig {
        public static final int blinkinPort = 3;
    }
}
