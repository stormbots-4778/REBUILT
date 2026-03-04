package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final SendableChooser<Command> autoChooser;

    private static final boolean USE_FIELD_RELATIVE = true;
    // The drivetrain rotation tapers off as it approaches the target angle.
    private static final double AUTOTARGET_CORRECTION_FACTOR = 2;

    private static final Translation2d GOAL_POSITION = FieldConfiguration.RED_GOAL_CENTER;

    public RobotContainer() {
        configureNamedCommands();
        m_drivetrain.setDefaultCommand(new RunCommand(this::drive, m_drivetrain));
        m_shooter.setDefaultCommand(
                m_shooter.shootWithDistance(
                        () -> distanceFromCoordinate(GOAL_POSITION),
                        m_controller.x().or(m_controller.rightTrigger())));
        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    private void configureNamedCommands() {
        // NamedCommands.registerCommand("Activate Hood",
        // m_shooter.setHood(HOOD_POSITION_FOR_AUTO));
        // NamedCommands.registerCommand("Deactivate Hood", m_shooter.setHood(0));
        // NamedCommands.registerCommand("Feed Shooter", m_shooter.feed());
        // NamedCommands.registerCommand("Start Intaking", m_intake.startIntaking());
        // NamedCommands.registerCommand("Stop Intaking", m_intake.stopIntaking());
        // NamedCommands.registerCommand("Deploy Intaker", m_intake.deploy());
        // NamedCommands.registerCommand("Retract Intaker", m_intake.retract());
    }

    private void configureButtonBindings() {
        m_controller.leftTrigger().whileTrue(m_intake.intake(m_shooter));
        m_controller.rightBumper().whileTrue(m_intake.agitate(m_shooter));
        // while right trigger and not right bumper (agitate cancels feed)
        m_controller.rightTrigger().and(m_controller.rightBumper().negate())
                .whileTrue(m_shooter.feed(m_intake));

        m_controller.back().onTrue(m_drivetrain.resetIMU);
        // m_controller.povRight().onTrue(m_intake.deploy);
        // m_controller.povLeft().onTrue(m_intake.retract);
    }

    /**
     * I like to move it move it
     */
    private void drive() {
        m_vision.getPoses().useOn(m_drivetrain);

        final var xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * DriveConfig.maxSpeed;
        final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * DriveConfig.maxSpeed;

        double rotation = m_controller.x().getAsBoolean() ? autotargetRotation() : driverRotation();

        m_drivetrain.drive(xSpeed, ySpeed, rotation, USE_FIELD_RELATIVE);
    }

    /**
     * How much do y'all want me to turn by?
     */
    private double driverRotation() {
        return -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                * DriveConfig.maxAngularSpeed;
    }

    /**
     * How much does autotargeting want me to turn by?
     */
    private double autotargetRotation() {
        return rotationToCoordinate(GOAL_POSITION)
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
        var robotHeading = Math.toRadians(m_drivetrain.getHeadingDegrees());
        return MathUtil.angleModulus(targetAngle - robotHeading);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
