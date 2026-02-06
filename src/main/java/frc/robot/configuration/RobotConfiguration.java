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

        public static final double wheelDiameter = 0.07874;
        public static final double wheelCircumferenceMeters = wheelDiameter * Math.PI;
    }

    public static final class DriveConfig {
        public static final double maxSpeed = 0.67;
        public static final double maxAngularSpeed = Math.PI / 2;

        private static final int drivingMotorPinionTeeth = 15;
        public static final double drivingMotorReduction = (45.0 * 20) / (drivingMotorPinionTeeth * 15);

        public static final double drivingMotorFreeSpeedRps = 6784 / 60; // neo motor is 5676
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
            public static final int backLeftDriveCAN = 5;
            public static final int backRightDriveCAN = 7;

            public static final int frontLeftTurnCAN = 2;
            public static final int frontRightTurnCAN = 4;
            public static final int backLeftTurnCAN = 6;
            public static final int backRightTurnCAN = 8;
        }

        public static final class MAXSwerveModule {
            public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
            public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

            static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double drivingFactor = ChassisConfig.wheelDiameter * Math.PI
                        / drivingMotorReduction;
                double turningFactor = 2 * Math.PI;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80);
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.04, 0, 0)
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
            }
        }
    }
}
