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
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    public void setPivot(double value) {
        pivotController.setSetpoint(value, ControlType.kPosition);
    }

    /*
     * true = 0;
     * false = 15;
     */
    private boolean statusDeploy = true; 

    // public Command deploy() {
    //     return runOnce(() -> {setPivotCurrent(10);})
    //        .andThen(run(() -> setPivot(10)).withTimeout(1)).finallyDo(() -> pivotMotor.stopMotor());
    // }

    // public Command semiDeploy(){
    //     return runOnce(() -> {setPivotCurrent(55); setPivotPGain(1); setPivotIGain(0.1);})
    //        .andThen(run(() -> setPivot(2)));
    // }

    // public Command retract() {
    //     return runOnce(() -> {setPivotCurrent(55); setPivotPGain(1); setPivotIGain(0.1);})
    //        .andThen(run(() -> setPivot(0)));
    // }

    public Command deploy() {
        return run(() -> setPivot(8)).withTimeout(0.5).finallyDo(() -> pivotMotor.stopMotor());
    }

    public Command semiDeploy(){
        return run(() -> setPivot(4));
    }

    public Command retract() {
        return run(() -> setPivot(1.5));
    }
    

    public Command agitate(Intake intake){
        return Commands.repeatingSequence(
            semiDeploy().withTimeout(0.5),
            deploy().withTimeout(0.5)).alongWith(intake.intake2()).finallyDo(() -> setPivot(10));
    }

    public void setPivotCurrent(int value) {
        IntakeConfig.pivotConfig.smartCurrentLimit(value);
    }

    public void setPivotPGain(double val){
        IntakeConfig.pivotConfig.closedLoop.p(val);
    }

    public void setPivotIGain(double val){
        IntakeConfig.pivotConfig.closedLoop.i(val);
    }

    public Command setPivotEncoder(){
        return runOnce(() -> {
            System.out.println("balls");
        });
    }

    // public Command defaultC(){
    //     return run(() -> defaultSequence());
    // }
    // public void defaultSequence(){
    //     if(statusDeploy){
    //         setPivotCurrent(0);
    //     }else{
    //         setPivotCurrent(15);
    //     }
    // }

    // public Command agitate() {
    //     return Commands.repeatingSequence(
    //             run(() -> agitators.set(AgitationConfig.activePosition)).withTimeout(AgitationConfig.repeatInterval),
    //             run(() -> agitators.set(AgitationConfig.inactivePosition)).withTimeout(AgitationConfig.repeatInterval))
    //             .finallyDo(() -> agitators.set(AgitationConfig.inactivePosition));
    // }

    // public Command deploy() {
    // return run(() -> pivotMotor.setVoltage(-3)).withTimeout(2)
    // .andThen(() -> pivotMotor.setVoltage(0));
    // }
}
