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

    public Command intake() {
        return runEnd(() -> intakerController.setSetpoint(IntakeConfig.INTAKER_SPEED, ControlType.kVelocity),
                this::stop);
    }

    public Command intakeWithIndexer(Feeder feeder) {
        return runEnd(() -> intakerController.setSetpoint(IntakeConfig.INTAKER_SPEED, ControlType.kVelocity),
                this::stop).deadlineFor(feeder.outtakeIndexer());
    }

    public Command outtake(Feeder feeder) {
        return runEnd(() -> intakerController.setSetpoint(-IntakeConfig.INTAKER_SPEED, ControlType.kVelocity),
                this::stop).deadlineFor(feeder.pushOut());
    }

    private void stop() {
        intakerController.setSetpoint(0, ControlType.kVelocity);
    }
}
