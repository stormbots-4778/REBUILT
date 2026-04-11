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

    public void setPivot(double value) {
        pivotController.setSetpoint(value, ControlType.kPosition);
    }

    public Command deploy() {
        return run(() -> setPivot(8)).withTimeout(0.5).finallyDo(() -> pivotMotor.stopMotor());
    }

    public Command semiDeploy() {
        return run(() -> setPivot(4));
    }

    public Command retract() {
        return run(() -> setPivot(1.5));
    }

    public Command agitate(Intake intake) {
        return Commands.repeatingSequence(
                semiDeploy().withTimeout(0.5),
                deploy().withTimeout(0.5)).alongWith(intake.intake()).finallyDo(() -> setPivot(10));
    }

    public Command setPivotEncoder() {
        return runOnce(() -> {
            System.out.println("balls");
        });
    }
}
