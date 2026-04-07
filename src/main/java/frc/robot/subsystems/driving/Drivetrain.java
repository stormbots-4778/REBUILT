// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driving;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
            DriveConfig.ModuleConfigs.frontLeftCAO,
            RobotConfiguration.DriveConfig.MAXSwerveModule.turningConfig);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.frontRightDriveCAN,
            DriveConfig.ModuleConfigs.frontRightTurnCAN,
            DriveConfig.ModuleConfigs.frontRightCAO,
            RobotConfiguration.DriveConfig.MAXSwerveModule.turningConfigFR);
    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.backLeftDriveCAN,
            DriveConfig.ModuleConfigs.backLeftTurnCAN,
            DriveConfig.ModuleConfigs.backLeftCAO,
            RobotConfiguration.DriveConfig.MAXSwerveModule.turningConfig);
    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
            DriveConfig.ModuleConfigs.backRightDriveCAN,
            DriveConfig.ModuleConfigs.backRightTurnCAN,
            DriveConfig.ModuleConfigs.backRightCAO,
            RobotConfiguration.DriveConfig.MAXSwerveModule.turningConfig);

    private final Pigeon2 m_gyro = new Pigeon2(DriveConfig.pigeonCAN);
    private final StructPublisher<Pose2d> positionPublisher;

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConfig.kinematics,
            getGyroYaw(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            }, new Pose2d());

    double[] m_gyroAccels = { 0, 0 };

    private boolean hasLocalizedWithVision = false; // cancel odometry if we havent localized yet

    /** Let's roll. */
    public Drivetrain() {
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.getYaw().setUpdateFrequency(100);
        m_gyro.getGravityVectorZ().setUpdateFrequency(100);
        m_gyro.setYaw(0.0);

        initAutonomousStuff();
        positionPublisher = NetworkTableInstance.getDefault().getStructTopic("4778BotPose", Pose2d.struct).publish();
    }

    private void initAutonomousStuff() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(100, 0, 2),// 150, 0, 0.01
                        new PIDConstants(37, 0.15, 0)), //100, 1, 0
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    /**
     * Tell me where I am.
     */
    public void usePoseEstimate(Pose2d pose, double timestamp) {
        if (!hasLocalizedWithVision)
            m_poseEstimator.resetPosition(getGyroYaw(), new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            }, pose);
        hasLocalizedWithVision = true;
        m_poseEstimator.addVisionMeasurement(pose, timestamp);
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
        m_poseEstimator.update(
                getGyroYaw(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        m_gyroAccels[0] = m_gyro.getAccelerationX().getValueAsDouble();
        m_gyroAccels[1] = m_gyro.getAccelerationY().getValueAsDouble();

        positionPublisher.set(getPose());
    }

    /**
     * I'm looking forward!
     */
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
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /**
     * Where am I?
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Tell me where I am.
     */
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
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
     * @param limitSpeed    Whether to use a limited maximum speed.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean limitSpeed, boolean fieldRelative) {
        double maxSpeed = limitSpeed ? DriveConfig.maxSpeedLimited : DriveConfig.maxSpeed;

        // Convert the commanded speeds into the correct units for my drivetrain.
        double xSpeedDelivered = xSpeed * maxSpeed;
        double ySpeedDelivered = ySpeed * maxSpeed;
        double rotDelivered = rot * DriveConfig.maxAngularSpeed;
        var swerveModuleStates = DriveConfig.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                getGyroYaw())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, maxSpeed);
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
     * Where am I looking in degrees?
     *
     * @return My heading in degrees, from -180 to 180
     */
    public double getHeadingDegrees() {
        return m_gyro.getYaw().getValueAsDouble();
    }
}