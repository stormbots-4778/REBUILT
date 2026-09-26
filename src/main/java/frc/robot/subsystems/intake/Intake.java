package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.IntakeConfig;
import frc.robot.subsystems.feeding.Feeder;

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

    private final SparkMax pivotMotor = setupSpark(IntakeConfig.pivotCAN, IntakeConfig.pivotConfig);
    private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private void setPivot(double pos) {
        pivotController.setSetpoint(pos, ControlType.kPosition);
    }

    public Command retract() {
        return run(() -> setPivot(0));
    }

    public Command deploy() {
        return run(() -> setPivot(9)).withTimeout(0.25).finallyDo(() -> pivotMotor.disable());
    }

    public Command agitate() {
        return Commands.repeatingSequence(
                run(() -> setPivot(4)).withTimeout(0.25),
                run(() -> setPivot(8)).withTimeout(0.25));
    }

    public Command intake() {
        return runEnd(() -> intakerController.setSetpoint(IntakeConfig.INTAKER_SPEED, ControlType.kVelocity),
                this::stop);
    }

    public Command outtake(Feeder feeder) {
        return runEnd(() -> intakerController.setSetpoint(-IntakeConfig.INTAKER_SPEED, ControlType.kVelocity),
                this::stop).deadlineFor(feeder.pushOut());
    }

    private void stop() {
        intakerController.setSetpoint(0, ControlType.kVelocity);
    }
}
