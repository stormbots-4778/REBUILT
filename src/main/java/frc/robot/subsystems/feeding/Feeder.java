package frc.robot.subsystems.feeding;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.configuration.RobotConfiguration.FeederConfig;;

public class Feeder extends SubsystemBase {
    private final SparkMax kickwheelMotor;
    private final SparkClosedLoopController kickwheelController;
    private final SparkFlex indexerMotor;
    private final SparkClosedLoopController indexerController;

    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController conveyorController;

    public Feeder() {
        kickwheelMotor = new SparkMax(FeederConfig.kickwheelCAN, MotorType.kBrushless);
        kickwheelMotor.configure(FeederConfig.kickwheelConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        kickwheelController = kickwheelMotor.getClosedLoopController();

        indexerMotor = new SparkFlex(FeederConfig.indexerCAN, MotorType.kBrushless);
        indexerMotor.configure(FeederConfig.indexerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        indexerController = indexerMotor.getClosedLoopController();

        conveyorMotor = new SparkMax(FeederConfig.conveyorCAN, MotorType.kBrushless);
        conveyorMotor.configure(FeederConfig.conveyorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        conveyorController = conveyorMotor.getClosedLoopController();
    }

    public void setIndexerVelocity(double velocity) {
        indexerController.setSetpoint(velocity, ControlType.kVelocity);
    }

    private static final double KICKWHEEL_AT_SPEED_THRESHOLD = 600;

    private boolean kickwheelAtSpeed() {
        return Math.abs(kickwheelMotor.getEncoder().getVelocity())
                + KICKWHEEL_AT_SPEED_THRESHOLD > FeederConfig.kickwheelSpeed;
    }

    private void setKickwheel(double velocity) {
        kickwheelController.setSetpoint(velocity, ControlType.kVelocity);
    }

    public Command outtakeIndexer() {
        return runEnd(() -> setIndexerVelocity(FeederConfig.indexerVelocityOuttake), () -> setIndexerVelocity(0));
    }

    private void setConveyor(double speed) {
        conveyorController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public Command feed() {
        return run(() -> setKickwheel(FeederConfig.kickwheelSpeed))
                .alongWith(
                        new WaitUntilCommand(this::kickwheelAtSpeed).andThen(
                                new RunCommand(() -> {
                                    setIndexerVelocity(FeederConfig.indexerVelocityIntake);
                                    setConveyor(FeederConfig.conveyorSpeedShoot);
                                })))
                .finallyDo(() -> {
                    setIndexerVelocity(0);
                    setKickwheel(0);
                    setConveyor(0);
                });
    }

    public Command pullIn() {
        return runEnd(() -> {
            setConveyor(FeederConfig.conveyorSpeed);
        }, () -> {
            setConveyor(0);
        }).deadlineFor(outtakeIndexer());
    }

    public Command pushOut() {
        return runEnd(() -> {
            setConveyor(-FeederConfig.conveyorSpeed);
            setIndexerVelocity(FeederConfig.indexerVelocityOuttake);
        }, () -> {
            setConveyor(0);
            setIndexerVelocity(0);
        });
    }
}
