// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driving;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration;
import frc.robot.configuration.RobotConfiguration.DriveConfig;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.frontLeftDriveCAN,
            DriveConfig.ModuleConfigs.frontLeftTurnCAN,
            DriveConfig.ModuleConfigs.frontLeftCAO);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.frontRightDriveCAN,
            DriveConfig.ModuleConfigs.frontRightTurnCAN,
            DriveConfig.ModuleConfigs.frontRightCAO);
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.backLeftDriveCAN,
            DriveConfig.ModuleConfigs.backLeftTurnCAN,
            DriveConfig.ModuleConfigs.backLeftCAO);
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.backRightDriveCAN,
            DriveConfig.ModuleConfigs.backRightTurnCAN,
            DriveConfig.ModuleConfigs.backRightCAO);

    private final Pigeon2 m_gyro = new Pigeon2(DriveConfig.pigeonCAN);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConfig.kinematics,
            getGyroYaw(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    double[] m_gyroAccels = { 0, 0 };

    /** Creates a new DriveSubsystem. */
    public Drivetrain() {
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.getYaw().setUpdateFrequency(100);
        m_gyro.getGravityVectorZ().setUpdateFrequency(100);
        m_gyro.setYaw(0.0);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return RobotConfiguration.DriveConfig.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(DriveConfig.kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public boolean detectedCollision() {
        return Math.abs(m_gyroAccels[0]) + Math.abs(m_gyroAccels[1]) >= 3;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getGyroYaw(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        m_gyroAccels[0] = m_gyro.getAccelerationX().getValueAsDouble();
        m_gyroAccels[1] = m_gyro.getAccelerationY().getValueAsDouble();
    }

    public Command resetIMU() {
        return runOnce(
                () -> {
                    m_gyro.reset();
                });
    }

    /**
     * Returns the gyros yaw as a Rotation2d
     * 
     * @return the yaw
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(getGyroDouble());
    }

    /**
     * Returns the gyros yaw as a double
     * 
     * @return the yaw
     */
    public double getGyroDouble() {
        return m_gyro.getYaw().getValueAsDouble();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                getGyroYaw(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                },
                pose);
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
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConfig.maxSpeed;
        double ySpeedDelivered = ySpeed * DriveConfig.maxSpeed;
        double rotDelivered = rot * DriveConfig.maxAngularSpeed;
        var swerveModuleStates = DriveConfig.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                getGyroYaw())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConfig.maxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConfig.maxSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState()
        };
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_backLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getGyroDouble();
    }
}