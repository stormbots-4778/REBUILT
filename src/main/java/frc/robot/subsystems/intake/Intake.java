package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.IntakeConfig;
import frc.robot.subsystems.shooting.Shooters;

public class Intake extends SubsystemBase {
    private static SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    private final SparkMax intakerMotor = setupSpark(
            IntakeConfig.intakerCAN,
            IntakeConfig.intakerConfig);
    private final SparkClosedLoopController intakerController = intakerMotor.getClosedLoopController();

    private final SparkMax pivotMotor = setupSpark(
            IntakeConfig.pivotCAN,
            IntakeConfig.pivotConfig);

    private final SparkMax conveyorMotor = setupSpark(
            IntakeConfig.conveyorCAN,
            IntakeConfig.conveyorConfig);
    private final SparkClosedLoopController conveyorController = conveyorMotor.getClosedLoopController();

    public Command deploy() {
        return run(() -> pivotMotor.setVoltage(-1)).withTimeout(2)
                .andThen(() -> pivotMotor.setVoltage(0));
    }

    public Command startIntaking() {
        return runOnce(this::start);
    }

    public Command stopIntaking() {
        return runOnce(this::stop);
    }

    public Command intake(Shooters shooters) {
        return runEnd(this::start, this::stop).deadlineFor(shooters.outtakeIndexer());
    }

    public Command outtake(Shooters shooters) {
        return runEnd(this::reverse, this::stop).deadlineFor(shooters.outtakeIndexer());
    }

    private void setConveyor(double speed) {
        conveyorController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public Command agitate(Shooters shooters) {
        return runEnd(() -> {
            setConveyor(IntakeConfig.CONVEYOR_SPEED_INTAKE);
        }, this::stop).deadlineFor(shooters.outtakeIndexer());
    }

    public Command runConveyorShoot() {
        return runEnd(
                () -> {
                    setConveyor(-IntakeConfig.CONVEYOR_SPEED_SHOOT);
                },
                this::stopConveyor);
    }

    private void start() {
        intakerController.setSetpoint(-IntakeConfig.INTAKER_SPEED, ControlType.kVelocity);
        setConveyor(-IntakeConfig.CONVEYOR_SPEED_INTAKE);
    }

    private void reverse() {
        intakerController.setSetpoint(IntakeConfig.INTAKER_SPEED, ControlType.kVelocity);
        setConveyor(IntakeConfig.CONVEYOR_SPEED_INTAKE);
    }

    public void stopConveyor() {
        setConveyor(0);
    }

    private void stop() {
        intakerController.setSetpoint(0, ControlType.kVelocity);
        stopConveyor();
    }
}
