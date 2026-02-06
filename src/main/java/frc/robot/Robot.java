// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.configuration.RobotConfiguration.DriveConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Robot extends TimedRobot {
    private final XboxController m_controller = new XboxController(0);
    private final Drivetrain m_drivetrain = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final double AUTOTARGET_CORRECTION_FACTOR = 5;
    private final Translation2d TESTING_FIXED_POINT = new Translation2d(0, -4);

    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * DriveConfig.maxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * DriveConfig.maxSpeed;

        final var rot = getNewRotation();
        System.out.println("rotation: " + rot);

        m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /**
     * Where I should rotate to this period.
     */
    private double getNewRotation() {
        // B = autotarget; guard clause
        if (!m_controller.getBButton()) {
            // Get the rate of angular rotation. We are inverting this because we want a
            // positive value when we pull to the left (remember, CCW is positive in
            // mathematics). Xbox controllers return positive values when you pull to
            // the right by default.
            return -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                    * DriveConfig.maxAngularSpeed;
        }

        final var triangle = TESTING_FIXED_POINT.minus(m_drivetrain.getPose().getTranslation());
        final var tan = Math.atan2(triangle.getY(), triangle.getX());

        final var heading = Math.toRadians(m_drivetrain.getHeading());
        return (tan - heading) * AUTOTARGET_CORRECTION_FACTOR;
    }
}
