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
import frc.robot.subsystems.shooting.Shooters;

public class Intake extends SubsystemBase {
    private static SparkMax setupSpark(int can, SparkMaxConfig config) {
        var m = new SparkMax(can, MotorType.kBrushless);
        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        return m;
    }

    private final SparkMax intakerMotor = setupSpark(
            RobotConfiguration.IntakeConfig.intakerCAN,
            RobotConfiguration.IntakeConfig.intakerConfig);
    private final SparkClosedLoopController intakerController = intakerMotor.getClosedLoopController();

    private final SparkMax pivotMotor = setupSpark(
            RobotConfiguration.IntakeConfig.pivotCAN,
            RobotConfiguration.IntakeConfig.pivotConfig);
    private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
    private static final double DEPLOYED_POSITION = 1;
    private static final double RETRACTED_POSITION = 0;

    private final SparkMax conveyorMotor = setupSpark(
            RobotConfiguration.IntakeConfig.conveyorCAN,
            RobotConfiguration.IntakeConfig.conveyorConfig);
    private final SparkClosedLoopController conveyorController = conveyorMotor.getClosedLoopController();

    public Command deploy = runOnce(() -> pivotController.setSetpoint(DEPLOYED_POSITION, ControlType.kPosition));
    public Command retract = runOnce(() -> pivotController.setSetpoint(RETRACTED_POSITION, ControlType.kPosition));
    public Command startIntaking = runOnce(this::start);
    public Command stopIntaking = runOnce(this::stop);

    public Command intake(Shooters shooters) {
        return runEnd(this::start, this::stop).deadlineFor(shooters.outtakeIndexer());
    }

    public Command agitate(Shooters shooters) {
        return runEnd(() -> {
            conveyorController.setSetpoint(RobotConfiguration.IntakeConfig.CONVEYOR_SPEED_INTAKE,
                    ControlType.kVelocity);
        }, this::stop).deadlineFor(shooters.outtakeIndexer());
    }

    public Command runConveyorShoot = runEnd(
            () -> conveyorController.setSetpoint(-RobotConfiguration.IntakeConfig.CONVEYOR_SPEED_SHOOT,
                    ControlType.kVelocity),
            this::stopConveyor);

    private void start() {
        intakerController.setSetpoint(-RobotConfiguration.IntakeConfig.INTAKER_SPEED, ControlType.kVelocity);
        conveyorController.setSetpoint(-RobotConfiguration.IntakeConfig.CONVEYOR_SPEED_INTAKE, ControlType.kVelocity);
    }

    public void stopConveyor() {
        conveyorController.setSetpoint(0, ControlType.kVelocity);
    }

    private void stop() {
        intakerController.setSetpoint(0, ControlType.kVelocity);
        stopConveyor();
    }
}
