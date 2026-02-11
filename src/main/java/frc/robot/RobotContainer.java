package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.FieldConfiguration;
import frc.robot.configuration.RobotConfiguration.AutotargetingConfig;
import frc.robot.configuration.RobotConfiguration.DriveConfig;
import frc.robot.subsystems.driving.Drivetrain;
import frc.robot.vision.Limelight;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Limelight m_limelight = new Limelight("limelight-silly");

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final SendableChooser<Command> autoChooser;

    private static final boolean USE_FIELD_RELATIVE = true;

    public RobotContainer() {
        m_drivetrain.setDefaultCommand(new RunCommand(this::drive, m_drivetrain));
        m_controller.y().onTrue(new InstantCommand(this::relocalize, m_drivetrain));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    /**
     * I like to move it move it
     */
    private void drive() {
        relocalize();

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * DriveConfig.maxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * DriveConfig.maxSpeed;

        double rotation;
        if (!m_controller.b().getAsBoolean()) {
            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            rotation = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                    * DriveConfig.maxAngularSpeed;
        } else {
            rotation = rotationToCoordinate(FieldConfiguration.RED_GOAL_CENTER)
                    * AutotargetingConfig.taperCorrectionFactor;
        }

        m_drivetrain.drive(xSpeed, ySpeed, rotation, USE_FIELD_RELATIVE);
    }

    /**
     * What do I need to rotate by to point to a coordinate on the field?
     */
    private double rotationToCoordinate(Translation2d fieldCoord) {
        final var triangle = fieldCoord.minus(m_drivetrain.getPose().getTranslation());

        var tan = Math.atan2(triangle.getY(), triangle.getX());
        tan = normalizeRadian(tan);

        var heading = Math.toRadians(m_drivetrain.getHeadingDegrees());
        heading = normalizeRadian(heading);

        return tan - heading;
    }

    private void relocalize() {
        Pose2d newPose = m_limelight.getBotPose(m_drivetrain.getHeadingDegrees());
        if (newPose == null)
            return;
        m_drivetrain.setPose(newPose);
    }

    public static final double TWOPI = Math.PI * 2;

    /**
     * Takes a radian value and ensures it's in the range [-PI, PI] ([-180deg,
     * 180deg])
     */
    private double normalizeRadian(double rad) {
        return ((rad % TWOPI) + TWOPI) % TWOPI - Math.PI;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
