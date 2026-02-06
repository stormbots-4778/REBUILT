package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.configuration.RobotConfiguration.AutotargetingConfig;
import frc.robot.configuration.RobotConfiguration.DriveConfig;
import frc.robot.subsystems.driving.Drivetrain;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
    public record DriveCommands(double x, double y, double r) {
    }

    private final XboxController m_controller = new XboxController(0);
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Limelight m_limelight = new Limelight(Limelight.PositionTeam.BLUE, "limelight-silly");

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public RobotContainer() {
        m_drivetrain.setDefaultCommand(new RunCommand(() -> {
            var dc = getDriveCommands();
            m_drivetrain.drive(dc.x, dc.y, dc.r, false);
        }, m_drivetrain));
    }

    private DriveCommands getDriveCommands() {
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
        if (!m_controller.getBButton()) {
            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            rotation = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                    * DriveConfig.maxAngularSpeed;
        } else {
            rotation = m_limelight.tagAngle() * AutotargetingConfig.taperCorrectionFactor;
        }

        return new DriveCommands(xSpeed, ySpeed, rotation);
    }
}
