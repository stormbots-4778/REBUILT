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
import frc.robot.configuration.RobotConfiguration;

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

        public DoubleShooterSparks(int can1, int can2, SparkMaxConfig config) {
            motor1 = setupSpark(can1, config);
            motor1Controller = motor1.getClosedLoopController();
            motor2 = setupSpark(can2, config);
            motor2Controller = motor2.getClosedLoopController();
        }

        public void set(double value) {
            motor1Controller.setSetpoint(-value, ControlType.kMAXMotionVelocityControl);
            motor2Controller.setSetpoint(value, ControlType.kMAXMotionVelocityControl);
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
            servo2.setAngle(90 - degrees); // left
            servo1.setAngle(90 - -degrees); // right
        }
    }

    private final DoubleShooterSparks flywheels;
    private final DoubleServo hoods;

    public Shooters() {
        flywheels = new DoubleShooterSparks(
                RobotConfiguration.ShooterConfig.flywheel1CAN,
                RobotConfiguration.ShooterConfig.flywheel2CAN,
                RobotConfiguration.ShooterConfig.flywheelConfig);
        flywheels.set(500);

        hoods = new DoubleServo(
                RobotConfiguration.ShooterConfig.hood1Port,
                RobotConfiguration.ShooterConfig.hood2Port);
    }

    public Command useDistance(DoubleSupplier distanceSupplier, BooleanSupplier enableHood,
            BooleanSupplier enableFlywheel) {
        return run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double shootval = ShootingDistanceTables.shooter.get(distance);
            flywheels.set(enableFlywheel.getAsBoolean() ? shootval + shooterOffset : 2000);
            hoods.setAngle(enableHood.getAsBoolean() ? ShootingDistanceTables.hood.get(distance) + hoodOffset : 0);
        });
    }

    public Command useDistance(double distance) {
        return useDistance(() -> distance, () -> true, () -> true);
    }

    private double hoodOffset = 0;

    public void incrementHoodOffset() {
        hoodOffset += 1.0;
    }

    public void decrementHoodOffset() {
        hoodOffset -= 1.0;
    }

    public double returnHoodOffset() {
        return hoodOffset;
    }

    private double shooterOffset = 0;

    public void incrementShooterOffset() {
        shooterOffset += 25;
    }

    public void decrementShooterOffset() {
        shooterOffset -= 25;
    }

    public double returnShooterOffset() {
        return shooterOffset;
    }

    // public Command setPower(double shooterSpeed, double hoodAngle) {
    // return run(() -> {
    // flywheels.set(shooterSpeed);
    // hoods.setAngle(hoodAngle);
    // });
    // }
}
