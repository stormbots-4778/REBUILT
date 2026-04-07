package frc.robot.subsystems.feeding;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.configuration.RobotConfiguration;
import frc.robot.configuration.RobotConfiguration.IntakeConfig;

public class Feeder extends SubsystemBase {
    private final SparkMax kickwheelMotor;
    private final SparkClosedLoopController kickwheelController;
    private final SparkFlex indexerMotor;
    private final SparkClosedLoopController indexerController;
    private static final int INDEXER_GO = -6;

    private static final int INDEXER_VELOCITY_INTAKE = -4000; //1000
    private static final int INDEXER__VELOCITY_OUTAKE = 500;
    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController conveyorController;
    

    public Feeder() {
        kickwheelMotor = new SparkMax(RobotConfiguration.ShooterConfig.kickwheelCAN, MotorType.kBrushless);
        kickwheelMotor.configure(RobotConfiguration.ShooterConfig.kickwheelConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        kickwheelController = kickwheelMotor.getClosedLoopController();

        indexerMotor = new SparkFlex(RobotConfiguration.ShooterConfig.indexerCAN, MotorType.kBrushless);
        indexerMotor.configure(RobotConfiguration.ShooterConfig.indexerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters); 
        indexerController = indexerMotor.getClosedLoopController();

        conveyorMotor = new SparkMax(RobotConfiguration.IntakeConfig.conveyorCAN, MotorType.kBrushless);
        conveyorMotor.configure(RobotConfiguration.IntakeConfig.conveyorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        conveyorController = conveyorMotor.getClosedLoopController();
    }

    public void setIndexerVelocity(double velocity){
        indexerController.setSetpoint(velocity, ControlType.kVelocity);
    }

    private static final double KICKWHEEL_AT_SPEED_THRESHOLD = 600;

    private boolean kickwheelAtSpeed() {
        return Math.abs(kickwheelMotor.getEncoder().getVelocity())
                + KICKWHEEL_AT_SPEED_THRESHOLD > RobotConfiguration.ShooterConfig.KICKWHEEL_SPEED;
    }

    private void setKickwheel(double velocity) {
        kickwheelController.setSetpoint(velocity, ControlType.kVelocity);
    }

    public Command outtakeIndexer() {
        return runEnd(() -> {setIndexerVelocity(INDEXER__VELOCITY_OUTAKE);}, () -> {setIndexerVelocity(0);});
    }

    private void setConveyor(double speed) {
        conveyorController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public Command feed() {
        return run(() -> setKickwheel(RobotConfiguration.ShooterConfig.KICKWHEEL_SPEED))
                .alongWith(
                        new WaitUntilCommand(this::kickwheelAtSpeed).andThen(
                                new RunCommand(() -> {
                                    // setVelocity(indexerController,
                                    // -RobotConfiguration.ShooterConfig.INDEXER_SPEED);
                                    // indexerMotor.setVoltage(INDEXER_GO);
                                    setIndexerVelocity(INDEXER_VELOCITY_INTAKE);
                                    setConveyor(IntakeConfig.CONVEYOR_SPEED_SHOOT);
                                })))
                .finallyDo(() -> {
                    // setVelocity(indexerController, 0);
                    setIndexerVelocity(0);
                    // indexerMotor.setVoltage(0);
                    setKickwheel(0);
                    // terminationProcess();
                    setConveyor(0);
                })
                
                ;
    }

    public Command pullIn() {
        return runEnd(() -> {
            setConveyor(IntakeConfig.CONVEYOR_SPEED_INTAKE);
        }, () -> {
            setConveyor(0);
        }).deadlineFor(outtakeIndexer());
    }

    public Command pushOut() {
        return runEnd(() -> {
            setConveyor(-IntakeConfig.CONVEYOR_SPEED_INTAKE);
            // indexerMotor.setVoltage(-INDEXER_GO);
            setIndexerVelocity(INDEXER__VELOCITY_OUTAKE);
        }, () -> {
            setConveyor(0);
            setIndexerVelocity(0);
            // indexerMotor.setVoltage(0);
        });
    }
}




    // private enum feederTermination{
    //     RESET_TIMER,
    //     REVERSE, 
    //     STOP;
    // } 
    // feederTermination state = feederTermination.RESET_TIMER;
    // Timer waitForReverse = new Timer();

    // private void terminationProcess() {
    //     switch(state){
    //         case RESET_TIMER:
    //             waitForReverse.reset();
    //             state = feederTermination.REVERSE;
    //             break;

    //         case REVERSE:
    //                 System.out.println("reversing");

    //                 indexerMotor.setVoltage(-INDEXER_GO);
    //                 setKickwheel(-RobotConfiguration.ShooterConfig.KICKWHEEL_SPEED);

    //                 if(waitForReverse.hasElapsed(2)){
    //                     state = feederTermination.STOP;
    //                 }
    //             break;
    //         case STOP:
    //                 System.out.println("stopped");

    //                 indexerMotor.setVoltage(0);
    //                 setKickwheel(0);
    //             break;   
    //     }
    // }