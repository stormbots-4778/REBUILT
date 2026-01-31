// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driving;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.RobotConfiguration;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxSpeed = Math.PI; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            RobotConfiguration.Drivetrain.ModuleConfigs.frontLeftDriveCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.frontLeftTurnCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.frontLeftCAO);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            RobotConfiguration.Drivetrain.ModuleConfigs.frontRightDriveCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.frontRightTurnCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.frontRightCAO);
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
            RobotConfiguration.Drivetrain.ModuleConfigs.backLeftDriveCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.backLeftTurnCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.backLeftCAO);
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
            RobotConfiguration.Drivetrain.ModuleConfigs.backRightDriveCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.backRightTurnCAN,
            RobotConfiguration.Drivetrain.ModuleConfigs.backRightCAO);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drivetrain() {
        m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot,
                                        m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }
}
