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

/** This is how I roll. */
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

    /** Let's roll. */
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
     * Where am I looking?
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Where am I?
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    /**
     * Tell me where I am.
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
     * Tell me to go.
     *
     * @param xSpeed        Speed in the x direction (forward).
     * @param ySpeed        Speed in the y direction (sideways).
     * @param rot           My angular rate.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for my drivetrain.
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
     * Set my wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Set my swerve modules' states.
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

    /** Tell me I'm centered. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_backLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backRight.resetEncoders();
    }

    /** Tell me I'm facing forwards. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Where am I looking?
     *
     * @return My heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getYaw().getValueAsDouble();
    }
}