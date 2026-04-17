package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.FieldConfiguration;
import frc.robot.configuration.RobotConfiguration.DriveConfig;
import frc.robot.subsystems.agitation.Agitator;
import frc.robot.subsystems.driving.Drivetrain;
import frc.robot.subsystems.feeding.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooting.Shooters;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer implements RunnableRobot {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Vision m_vision = new Vision();
    private final Shooters m_shooter = new Shooters();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();
    private final Agitator m_agitator = new Agitator();

    private final SendableChooser<Command> autoChooser;

    private static final boolean USE_FIELD_RELATIVE = true;
    // shoot distance override
    private static final double DISTANCE_OVERRIDE = 1.5;

    private Alliance m_alliance;
    private Translation2d m_goalPosition;
    private Translation2d m_aimAtGoalPosition; // for autoalign

    private StructPublisher<Translation2d> m_goalPositionPublisher;
    private StructPublisher<Translation2d> m_aimAtGoalPositionPublisher;
    private double autoAimMultiplier = 1.25;

    public RobotContainer() {
        m_goalPositionPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("4778GoalPosition", Translation2d.struct).publish();
        m_aimAtGoalPositionPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("4778AimAtGoalPosition", Translation2d.struct).publish();

        configureNamedCommands();
        setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));

        m_drivetrain.setDefaultCommand(new RunCommand(this::drive, m_drivetrain));
        m_vision.setDefaultCommand(new RunCommand(() -> m_vision.passIntoDrivetrain(m_drivetrain), m_vision));
        m_shooter.setDefaultCommand(
                m_shooter.useDistance(
                        this::shooterShootDistance,
                        m_controller.x()
                                .or(m_controller.rightTrigger(0.2))
                                .or(m_controller.a()),
                        m_controller.x().or(m_controller.rightTrigger(0.2))));

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    private void setAlliance(Alliance alliance) {
        m_alliance = alliance;
        m_goalPosition = FieldConfiguration.getGoalPosition(alliance);
        m_aimAtGoalPosition = m_goalPosition;
        m_vision.setAlliance(alliance);

        m_goalPositionPublisher.set(m_goalPosition);
    }

    private Command resetHeading() {
        return Commands.parallel(m_drivetrain.resetIMU(), m_vision.zeroLimelightIMU());
    }

    /**
     * How far I should ACTUALLY shoot.
     */
    private double shooterShootDistance() {
        if (m_controller.a().getAsBoolean()) {
            return DISTANCE_OVERRIDE;
        }
        return distanceFromCoordinate(m_aimAtGoalPosition);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Intake", m_intake.intakeWithIndexer(m_feeder));
        NamedCommands.registerCommand("Deploy Intaker", m_agitator.deploy().withTimeout(0.25));
        NamedCommands.registerCommand("Reset Heading", resetHeading());

        NamedCommands.registerCommand("Shoot Routine", Commands.deadline(
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        m_feeder.feed().withTimeout(10).alongWith(m_agitator.agitate(m_intake))),
                m_shooter.useDistance(this::shooterShootDistance, () -> true, () -> true)));

        /*
         * Below are the emergency shoot routines for blue right and left
         */

        NamedCommands.registerCommand("Shoot Routine BLUE RIGHT", Commands.deadline(
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        m_feeder.feed().withTimeout(10).alongWith(m_agitator.agitate(m_intake))),
                m_shooter.useDistance(2.3))); // put manual distance for RIGHT SIDE AUTO. i
                                              // think this value is ok

        NamedCommands.registerCommand("Shoot Routine BLUE LEFT", Commands.deadline(
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        m_feeder.feed().withTimeout(10).alongWith(m_agitator.agitate(m_intake))),
                m_shooter.useDistance(2.3))); // put manual distance for LEFT SIDE AUTO

        NamedCommands.registerCommand("DEPOT Shoot Routine", Commands.deadline(
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        m_feeder.feed().withTimeout(6.7).alongWith(m_agitator.agitate(m_intake))),
                m_shooter.useDistance(this::shooterShootDistance, () -> true, () -> true)));
    }

    private void configureButtonBindings() {
        m_controller.leftTrigger()
                .whileTrue(m_intake.intakeWithIndexer(m_feeder));

        m_controller.rightTrigger().whileTrue(Commands.parallel(
                m_agitator.agitate(m_intake),
                m_feeder.pushOut().withTimeout(0.5).andThen(m_feeder.feed())));

        // todo review
        m_controller.x().and(() -> Math.abs(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1)) == 0)
                .and(() -> Math.abs(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1)) == 0)
                .debounce(2)
                .whileTrue(Commands.run(() -> m_drivetrain.setX()));

        m_controller.leftBumper()
                .whileTrue(m_intake.outtake(m_feeder));

        m_controller.back().onTrue(resetHeading());

        m_controller.b().whileTrue(m_agitator.deploy());
        m_controller.y().whileTrue(m_agitator.retract());

        m_controller.leftStick().onTrue(new InstantCommand(
                () -> setAlliance(Alliance.Blue)));
        m_controller.rightStick().onTrue(new InstantCommand(
                () -> setAlliance(Alliance.Red)));

        m_controller.povUp().onTrue(Commands.runOnce(() -> m_shooter.incrementHoodOffset()));
        m_controller.povDown().onTrue(Commands.runOnce(() -> m_shooter.decrementHoodOffset()));

        m_controller.povRight().onTrue(Commands.runOnce(() -> m_shooter.incrementShooterOffset()));
        m_controller.povLeft().onTrue(Commands.runOnce(() -> m_shooter.decrementShooterOffset()));
    }

    /**
     * I like to move it move it
     */
    private void drive() {
        // System.out.println(shooterShootDistance());
        // System.out.println(m_shooter.returnHoodOffset());
        // System.out.println(m_shooter.returnShooterOffset());
        // System.out.println(DriverStation.getAlliance());

        setAlliance(DriverStation.getAlliance().orElse(Alliance.Red));
        m_aimAtGoalPosition = autoaimTarget();
        m_aimAtGoalPositionPublisher.set(m_aimAtGoalPosition);

        final var xSpeed = MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)
                * DriveConfig.maxSpeed;
        final var ySpeed = MathUtil.applyDeadband(m_controller.getLeftX(), 0.02)
                * DriveConfig.maxSpeed;

        boolean shouldAutoTarget = m_controller.x().getAsBoolean();
        double rotation = shouldAutoTarget ? autotargetRotation() : driverRotation();

        m_drivetrain.drive(xSpeed, ySpeed, rotation, shouldAutoTarget, USE_FIELD_RELATIVE);
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
        return rotationToCoordinate(m_aimAtGoalPosition) * 2;
    }

    /**
     * Where should I look to autoaim well?
     */
    private Translation2d autoaimTarget() {
        ChassisSpeeds botVelocityField = ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getChassisSpeeds(),
                m_drivetrain.getGyroYaw());
        Translation2d difference = new Translation2d(
                botVelocityField.vxMetersPerSecond * autoAimMultiplier,
                botVelocityField.vyMetersPerSecond * autoAimMultiplier);

        return m_alliance == Alliance.Blue ? m_goalPosition.plus(difference) : m_goalPosition.minus(difference);
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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
