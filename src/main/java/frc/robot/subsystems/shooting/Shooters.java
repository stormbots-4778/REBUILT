package frc.robot.subsystems.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.configuration.RobotConfiguration;
import frc.robot.subsystems.intake.Intake;

public class Shooters extends SubsystemBase {
    private static SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    private static class DoubleShooterSparks {
        private final SparkMax motor1;
        public final SparkClosedLoopController motor1Controller;
        private final SparkMax motor2;
        public final SparkClosedLoopController motor2Controller;
        private final ControlType controlType;

        public DoubleShooterSparks(int can1, int can2, SparkMaxConfig config, ControlType controlType) {
            motor1 = setupSpark(can1, config);
            motor1Controller = motor1.getClosedLoopController();
            motor2 = setupSpark(can2, config);
            motor2Controller = motor2.getClosedLoopController();
            this.controlType = controlType;
        }

        public void set(double value) {
            motor1Controller.setSetpoint(-value, controlType);
            motor2Controller.setSetpoint(value, controlType);
        }
    }

    private static class DoubleServo {
        private final Servo servo1;
        private final Servo servo2;

        public DoubleServo(int port1, int port2) {
            servo1 = new Servo(port1);
            servo2 = new Servo(port2);
        }

        public void setAngle(double degrees) {
            servo1.setAngle(90 - degrees);
            servo2.setAngle(90 - -degrees);
        }
    }

    private final DoubleShooterSparks flywheels;
    private final DoubleServo hoods;

    private final SparkMax kickwheelMotor;
    private final SparkClosedLoopController kickwheelController;
    private final SparkMax indexerMotor;
    private final SparkClosedLoopController indexerController;

    public Shooters() {
        flywheels = new DoubleShooterSparks(
                RobotConfiguration.ShooterConfig.flywheel1CAN,
                RobotConfiguration.ShooterConfig.flywheel2CAN,
                RobotConfiguration.ShooterConfig.flywheelConfig,
                ControlType.kVelocity);
        flywheels.set(500);

        hoods = new DoubleServo(
                RobotConfiguration.ShooterConfig.hood1Port,
                RobotConfiguration.ShooterConfig.hood2Port);

        kickwheelMotor = setupSpark(RobotConfiguration.ShooterConfig.kickwheelCAN,
                RobotConfiguration.ShooterConfig.kickwheelConfig);
        kickwheelController = kickwheelMotor.getClosedLoopController();

        indexerMotor = setupSpark(RobotConfiguration.ShooterConfig.indexerCAN,
                RobotConfiguration.ShooterConfig.indexerConfig);
        indexerController = indexerMotor.getClosedLoopController();
    }

    private void setVelocity(SparkClosedLoopController controller, double velocity) {
        controller.setSetpoint(velocity, ControlType.kVelocity);
    }

    private static final double KICKWHEEL_AT_SPEED_THRESHOLD = 750;

    private boolean kickwheelAtSpeed() {
        return Math.abs(kickwheelMotor.getEncoder().getVelocity())
                + KICKWHEEL_AT_SPEED_THRESHOLD > RobotConfiguration.ShooterConfig.KICKWHEEL_RPM;
    }

    public Command outtakeIndexer() {
        return run(() -> setVelocity(indexerController, -RobotConfiguration.ShooterConfig.INDEXER_KICKOUT_RPM));
    }

    public Command feed(Intake intake) {
        return run(() -> setVelocity(kickwheelController, -RobotConfiguration.ShooterConfig.KICKWHEEL_RPM))
                .alongWith(
                        new WaitUntilCommand(this::kickwheelAtSpeed).andThen(
                                () -> {
                                    setVelocity(indexerController,
                                            -RobotConfiguration.ShooterConfig.INDEXER_RPM);
                                }))
                .alongWith(intake.runConveyorShoot)
                .finallyDo(() -> {
                    setVelocity(indexerController, 0);
                    setVelocity(kickwheelController, 0);
                });
    }

    // for auto
    public Command runAtVelocity(double velocity) {
        return run(() -> flywheels.set(velocity));
    }

    public Command setHood(double angle) {
        return run(() -> hoods.setAngle(angle));
    }

    // for teleop
    public Command shootWithDistance(DoubleSupplier distanceSupplier, BooleanSupplier enableHood) {
        return run(() -> {
            double distance = distanceSupplier.getAsDouble();
            Double shootval = ShootingDistanceTables.shooter.get(distance);
            flywheels.set(shootval);

            if (enableHood.getAsBoolean()) {
                Double hoodval = ShootingDistanceTables.hood.get(distance);
                hoods.setAngle(hoodval);
            } else {
                hoods.setAngle(0);
            }
        });
    }
}
