package frc.robot.subsystems.shooting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.ShooterConfig;

public class Shooters extends SubsystemBase {
    private class ManyShooters {
        private record ShooterDetails(int can, SparkMaxConfig config) {
        }

        private record MotorAndController(SparkMax motor, SparkClosedLoopController controller,
                RelativeEncoder encoder) {
        }

        private int numShooters;
        private MotorAndController[] motors;

        private ManyShooters(ShooterDetails... details) {
            numShooters = details.length;

            motors = new MotorAndController[numShooters];
            for (int i = 0; i < numShooters; i++) {
                SparkMax motor = setupSpark(details[i].can, details[i].config);
                motors[i] = new MotorAndController(motor, motor.getClosedLoopController(), motor.getEncoder());
            }
        }

        public void set(double val) {
            for (int i = 0; i < numShooters; i++) {
                motors[i].controller.setSetpoint(val, ControlType.kVelocity);
            }
        }

        public double getVelocity() {
            return motors[0].encoder.getVelocity();
        }
    }

    private static SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    private final ManyShooters shooters = new ManyShooters(
            new ManyShooters.ShooterDetails(
                    ShooterConfig.shooterLeft1CAN, ShooterConfig.shooterLeft1Config),
            new ManyShooters.ShooterDetails(
                    ShooterConfig.shooterLeft2CAN, ShooterConfig.shooterLeft2Config),
            new ManyShooters.ShooterDetails(
                    ShooterConfig.shooterRight1CAN, ShooterConfig.shooterRight1Config),
            new ManyShooters.ShooterDetails(
                    ShooterConfig.shooterRight2CAN, ShooterConfig.shooterRight2Config));

    private final SparkMax hoodLeftMotor = setupSpark(ShooterConfig.hoodLeftCAN, ShooterConfig.hoodLeftConfig);
    private final SparkClosedLoopController hoodLeftController = hoodLeftMotor.getClosedLoopController();

    static {
        setupSpark(ShooterConfig.hoodRightCAN, ShooterConfig.hoodRightConfig);
    }

    private DoubleTopic shootingTopic = NetworkTableInstance.getDefault().getDoubleTopic("4778Shooting");
    private DoublePublisher shootingPublisher = shootingTopic.publish();

    public double shootSpeedOffset = 0;
    public double hoodOffset = 0;

    private void setShooter(double vel) {
        shooters.set(vel);
        shootingPublisher.set(shooters.getVelocity());
    }

    private void setHood(double position) {
        hoodLeftController.setSetpoint(position, ControlType.kPosition);
    }

    public Command useDistance(DoubleSupplier distanceSupplier, BooleanSupplier enableHood,
            BooleanSupplier enableFlywheel) {
        return run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double shootval = ShootingDistanceTables.shooter.get(distance);
            double shootCommand = enableFlywheel.getAsBoolean()
                    ? shootval + shootSpeedOffset
                    : ShooterConfig.SHOOTER_IDLE_SPEED;
            setShooter(shootCommand);

            double hoodCommand = enableHood.getAsBoolean()
                    ? ShootingDistanceTables.hood.get(distance) + hoodOffset
                    : 0;
            setHood(hoodCommand);
        });
    }

    public Command useDistance(double distance) {
        return useDistance(() -> distance, () -> true, () -> true);
    }
}
