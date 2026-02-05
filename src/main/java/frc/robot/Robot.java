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

    // Slew rate limiter to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_joystickXSlew = new SlewRateLimiter(3);
    private final SlewRateLimiter m_joystickYSlew = new SlewRateLimiter(3);
    private final SlewRateLimiter m_joystickRSlew = new SlewRateLimiter(3);

    private final Translation2d TEST_AUTOALIGN_COORD = new Translation2d(0, -4);

    @Override
    public void autonomousPeriodic() {
        driveWithJoystick(false);
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    /**
     * Use the controller to command my drivetrain.
     * 
     * @param fieldRelative
     */
    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. Invert this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_joystickXSlew.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * DriveConfig.maxSpeed;

        // Get the y speed or sideways/strafe speed. Invert this for a positive value
        // when pulling to the left -- by default, Xbox controllers return positive
        // values when
        // pulling to the right.
        final var ySpeed = -m_joystickYSlew.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * DriveConfig.maxSpeed;

        double rotateBy;
        if (shouldAutotarget()) {
            rotateBy = MathUtil.clamp(
                    botRotationToPoint(TEST_AUTOALIGN_COORD),
                    -DriveConfig.maxAngularSpeed,
                    DriveConfig.maxAngularSpeed);
        } else {
            rotateBy = -m_joystickRSlew.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                    * DriveConfig.maxAngularSpeed;
        }

        m_drivetrain.drive(xSpeed, ySpeed, rotateBy, fieldRelative);
    }

    /**
     * How much do I need to rotate by to turn to a point?
     * 
     * @param toPoint The point.
     */
    private double botRotationToPoint(Translation2d toPoint) {
        final var pose = m_drivetrain.getPose();
        final var heading = Math.toRadians(m_drivetrain.getHeading());

        final var ang = Math.atan2(toPoint.getY() - pose.getY(), toPoint.getX() - pose.getX());
        return ang - heading;
    }

    /**
     * Should I be autotargeting?
     */
    private boolean shouldAutotarget() {
        return m_controller.getBButton();
    }
}
