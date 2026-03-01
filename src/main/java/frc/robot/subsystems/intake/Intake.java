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
import frc.robot.configuration.RobotConfiguration;

public class Intake extends SubsystemBase {
    private final SparkMax intakerMotor;
    private final SparkClosedLoopController intakerController;
    private final static double INTAKER_SPEED = 600;

    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotController;
    private static final double DEPLOYED_POSITION = 1;
    private static final double RETRACTED_POSITION = 0;

    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController conveyorController;
    private final static double CONVEYOR_SPEED = 600;

    private SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    public Intake() {
        intakerMotor = setupSpark(
                RobotConfiguration.IntakeConfig.intakerCAN,
                RobotConfiguration.IntakeConfig.intakerConfig);
        intakerController = intakerMotor.getClosedLoopController();

        pivotMotor = setupSpark(
                RobotConfiguration.IntakeConfig.pivotCAN,
                RobotConfiguration.IntakeConfig.pivotConfig);
        pivotController = pivotMotor.getClosedLoopController();

        conveyorMotor = setupSpark(
                RobotConfiguration.IntakeConfig.conveyorCAN,
                RobotConfiguration.IntakeConfig.conveyorConfig);
        conveyorController = conveyorMotor.getClosedLoopController();
    }

    public Command deploy() {
        return runOnce(() -> pivotController.setSetpoint(DEPLOYED_POSITION, ControlType.kPosition));
    }

    public Command retract() {
        return runOnce(() -> pivotController.setSetpoint(RETRACTED_POSITION, ControlType.kPosition));
    }

    private void start(boolean reversed) {
        intakerController.setSetpoint(reversed ? -INTAKER_SPEED : INTAKER_SPEED, ControlType.kVelocity);
        conveyorController.setSetpoint(reversed ? -CONVEYOR_SPEED : CONVEYOR_SPEED, ControlType.kVelocity);
    }

    private void stop() {
        intakerController.setSetpoint(0, ControlType.kVelocity);
        conveyorController.setSetpoint(0, ControlType.kVelocity);
    }

    public Command startIntaking() {
        return runOnce(() -> start(false));
    }

    public Command stopIntaking() {
        return runOnce(() -> stop());
    }

    public Command intake() {
        return runEnd(() -> start(false), () -> stop());
    }

    public Command outtake() {
        return runEnd(() -> start(true), () -> stop());
    }
}
