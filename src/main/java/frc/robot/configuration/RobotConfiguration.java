package frc.robot.configuration;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RobotConfiguration {
    public static final class Drivetrain {
        public static final double kWheelDiameterMeters = 0.07874;

        private static final int kDrivingMotorPinionTeeth = 15;
        public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);

        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60; // neo motor is 5676
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

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
                double drivingFactor = kWheelDiameterMeters * Math.PI
                        / kDrivingMotorReduction;
                double turningFactor = 2 * Math.PI;
                double nominalVoltage = 12.0;
                double drivingVelocityFeedForward = nominalVoltage / kDriveWheelFreeSpeedRps;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.04, 0, 0)
                        .outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

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
