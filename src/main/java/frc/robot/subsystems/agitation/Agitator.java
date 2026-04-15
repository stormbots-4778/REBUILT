package frc.robot.subsystems.agitation;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.AgitationConfig;
import frc.robot.configuration.RobotConfiguration.IntakeConfig;
import frc.robot.subsystems.intake.Intake;

public class Agitator extends SubsystemBase {
    private static SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    private final SparkMax pivotMotor = setupSpark(
            IntakeConfig.pivotCAN,
            IntakeConfig.pivotConfig);
    private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

    private final SparkMax agitatorLeftMotor = setupSpark(AgitationConfig.agitatorLeftCAN,
            AgitationConfig.agitatorConfig);
    private final SparkClosedLoopController agitatorLeftController = agitatorLeftMotor.getClosedLoopController();
    private final SparkMax agitatorRightMotor = setupSpark(AgitationConfig.agitatorRightCAN,
            AgitationConfig.agitatorConfig);
    private final SparkClosedLoopController agitatorRightController = agitatorRightMotor.getClosedLoopController();

    public void setPivot(double value) {
        pivotController.setSetpoint(value, ControlType.kPosition);
    }

    private void pivotDown() {
        setPivot(8);
    }

    private void pivotSemi() {
        setPivot(4);
    }

    private void pivotUp() {
        setPivot(1.5);
    }

    private void pivotDisable() {
        pivotMotor.stopMotor();
    }

    public Command deploy() {
        return run(this::pivotDown).withTimeout(0.5).finallyDo(this::pivotDisable);
    }

    public Command semiDeploy() {
        return run(this::pivotSemi);
    }

    public Command retract() {
        return run(this::pivotUp);
    }

    private void agitatorsUp() {
        agitatorLeftController.setSetpoint(1.1, ControlType.kPosition);
        agitatorRightController.setSetpoint(-1.1, ControlType.kPosition);
    }

    private void agitatorsDown() {
        agitatorLeftController.setSetpoint(0, ControlType.kPosition);
        agitatorRightController.setSetpoint(0, ControlType.kPosition);
    }

    public Command agitate(Intake intake) {
        return Commands.parallel(
                Commands.repeatingSequence(
                        Commands.run(() -> agitatorsUp()).withTimeout(AgitationConfig.repeatIntervalFlippers),
                        Commands.run(() -> agitatorsDown()).withTimeout(AgitationConfig.repeatIntervalFlippers)),
                Commands.repeatingSequence(
                        Commands.run(() -> pivotSemi()).withTimeout(AgitationConfig.repeatIntervalPivot),
                        Commands.run(() -> pivotDown()).withTimeout(AgitationConfig.repeatIntervalPivot)))
                .finallyDo(this::agitatorsUp);
    }

    // do NOT remove this
    public Command setPivotEncoder() {
        return runOnce(() -> {
            System.out.println("balls");
        });
    }
}
