package frc.robot.subsystems.feeding;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.FeederConfig;;

public class Feeder extends SubsystemBase {
    private final SparkMax leftFeedMotor;
    private final SparkClosedLoopController leftFeedController;
    private final SparkMax rightFeedMotor;
    // no need for right feed controller, it follows left motor (check config)

    public double feedMotorSpeedOffset = 0;

    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController conveyorController;

    public Feeder() {
        leftFeedMotor = new SparkMax(FeederConfig.feedMotorLeftCAN, MotorType.kBrushless);
        leftFeedMotor.configure(FeederConfig.feedMotorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        leftFeedController = leftFeedMotor.getClosedLoopController();

        rightFeedMotor = new SparkMax(FeederConfig.feedMotorRightCAN, MotorType.kBrushless);
        rightFeedMotor.configure(FeederConfig.feedMotorRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        conveyorMotor = new SparkMax(FeederConfig.conveyorCAN, MotorType.kBrushless);
        conveyorMotor.configure(FeederConfig.conveyorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        conveyorController = conveyorMotor.getClosedLoopController();
    }

    public void setFeedMotor(double vel) {
        leftFeedController.setSetpoint(vel, ControlType.kVelocity);
    }

    public Command holdup() {
        return runEnd(() -> setFeedMotor(FeederConfig.feederSpeedSlow), () -> setFeedMotor(0));
    }

    private void setConveyor(double speed) {
        conveyorController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public Command feed() {
        return run(() -> {
            setFeedMotor(FeederConfig.feederSpeed + feedMotorSpeedOffset);
            setConveyor(FeederConfig.conveyorSpeedShoot);
        })
                .finallyDo(() -> {
                    setFeedMotor(0);
                    setConveyor(0);
                });
    }

    public Command pullIn() {
        return runEnd(() -> {
            setConveyor(FeederConfig.conveyorSpeed);
        }, () -> {
            setConveyor(0);
        }).deadlineFor(holdup());
    }

    public Command pushOut() {
        return runEnd(() -> {
            setConveyor(-FeederConfig.conveyorSpeed);
            setFeedMotor(FeederConfig.feederSpeedSlow);
        }, () -> {
            setConveyor(0);
            setFeedMotor(0);
        });
    }
}
