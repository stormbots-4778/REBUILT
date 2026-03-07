package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.FieldConfiguration;
import frc.robot.configuration.RobotConfiguration.DriveConfig;
import frc.robot.subsystems.driving.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooting.Shooters;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Vision m_vision = new Vision();
    private final Shooters m_shooter = new Shooters();
    private final Intake m_intake = new Intake();

    private final SendableChooser<Command> autoChooser;

    private static final boolean USE_FIELD_RELATIVE = true;
    // shoot distance override
    private static final double DISTANCE_OVERRIDE = 1.5;
    // The drivetrain rotation tapers off as it approaches the target angle.
    private static final double AUTOTARGET_CORRECTION_FACTOR = 2;

    private Alliance m_alliance;
    private Translation2d m_goalPosition;

    public RobotContainer() {
        configureNamedCommands();
        setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));

        m_drivetrain.setDefaultCommand(new RunCommand(this::drive, m_drivetrain));
        m_vision.setDefaultCommand(new RunCommand(() -> m_vision.passIntoDrivetrain(m_drivetrain), m_vision));
        m_shooter.setDefaultCommand(
                m_shooter.shootWithDistance(
                        this::shooterShootDistance,
                        this::shouldEnableHood));

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    private void setAlliance(Alliance alliance) {
        m_alliance = alliance;
        m_goalPosition = FieldConfiguration.getGoalPosition(alliance);
        System.out.println("Set alliance: " + m_alliance + ". Goal position = " + m_goalPosition);
    }

    /**
     * Can I put my hood up?
     */
    private boolean shouldEnableHood() {
        if (m_controller.x()
                .or(m_controller.rightTrigger())
                .or(m_controller.a()).getAsBoolean())
            return true;
        if (RobotState.isAutonomous())
            return true;

        return false;
    }

    /**
     * How far I should ACTUALLY shoot.
     */
    private double shooterShootDistance() {
        if (m_controller.a().getAsBoolean()) {
            return DISTANCE_OVERRIDE;
        }
        return distanceFromCoordinate(m_goalPosition);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Activate Hood", m_shooter.setHood(20));
        NamedCommands.registerCommand("Deactivate Hood", m_shooter.setHood(0));
        NamedCommands.registerCommand("Feed Shooter", m_shooter.feed(m_intake).withTimeout(3));
        NamedCommands.registerCommand("Feed Shooter Quick", m_shooter.feed(m_intake).withTimeout(1));
        NamedCommands.registerCommand("Start Intaking", m_intake.intake(m_shooter));
        NamedCommands.registerCommand("Agitate", m_intake.agitate(m_shooter).withTimeout(1));
        NamedCommands.registerCommand("Stop Intaking", m_intake.stopIntaking());
        NamedCommands.registerCommand("Deploy Intaker", m_intake.deploy());
    }

    private void configureButtonBindings() {
        m_controller.leftTrigger().and(m_controller.rightBumper().negate())
                .whileTrue(new RepeatCommand(m_intake.intake(m_shooter)));

        m_controller.rightBumper().whileTrue(new RepeatCommand(m_intake.agitate(m_shooter)));
        // while right trigger and not right bumper (agitate cancels feed)
        m_controller.rightTrigger().and(m_controller.rightBumper().negate())
                .whileTrue(m_shooter.feed(m_intake));

        // overrides intake and agitate
        m_controller.leftTrigger().and(m_controller.rightBumper())
                .whileTrue(new RepeatCommand(m_intake.outtake(m_shooter)));

        m_controller.back().onTrue(m_drivetrain.resetIMU);
        m_controller.b().onTrue(m_intake.deploy());

        m_controller.leftStick().onTrue(new InstantCommand(
                () -> setAlliance(Alliance.Blue)));
        m_controller.rightStick().onTrue(new InstantCommand(
                () -> setAlliance(Alliance.Red)));
    }

    /**
     * I like to move it move it
     */
    private void drive() {
        final var xSpeed = MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)
                * DriveConfig.maxSpeed;
        final var ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(), 0.02)
                * DriveConfig.maxSpeed;

        double rotation = m_controller.x().getAsBoolean() ? autotargetRotation() : driverRotation();
        m_drivetrain.drive(xSpeed, ySpeed, rotation, USE_FIELD_RELATIVE);
    }

    /**
     * How much do y'all want me to turn by?
     */
    private double driverRotation() {
        return -MathUtil.applyDeadband(m_controller.getRightX(), 0.02)
                * DriveConfig.maxAngularSpeed;
    }

    /**
     * How much does autotargeting want me to turn by?
     */
    private double autotargetRotation() {
        return (rotationToCoordinate(m_goalPosition))
                * AUTOTARGET_CORRECTION_FACTOR;
    }

    /**
     * How far away am I from a coordinate on the field?
     */
    private double distanceFromCoordinate(Translation2d fieldCoord) {
        final var triangle = fieldCoord.minus(m_drivetrain.getPose().getTranslation());
        return triangle.getNorm();
    }

    /**
     * What do I need to rotate by to point to a coordinate on the field?
     */
    private double rotationToCoordinate(Translation2d fieldCoord) {
        final var delta = fieldCoord.minus(m_drivetrain.getPose().getTranslation());
        var targetAngle = Math.atan2(delta.getY(), delta.getX());
        var headingOffset = m_alliance == Alliance.Blue ? 180 : 0;
        var robotHeading = Math.toRadians(m_drivetrain.getHeadingDegrees() + headingOffset);
        return MathUtil.angleModulus(targetAngle - robotHeading);
    }

    private Command basicShootAuto = Commands.sequence(
            m_intake.deploy(),
            new RepeatCommand(Commands.sequence(
                    m_shooter.feed(m_intake).withTimeout(3),
                    m_intake.agitate(m_shooter).withTimeout(1))));

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return basicShootAuto;
    }
}
