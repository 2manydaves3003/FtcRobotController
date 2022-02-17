package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveTrain extends SubsystemBase {

    Motor m_frontLeftMotor;
    Motor m_frontRightMotor;
    Motor m_backLeftMotor;
    Motor m_backRightMotor;

    ChassisSpeeds m_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    Rotation2d m_gyroAngle = new Rotation2d(0.0);

    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation);

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight)
    {
        m_frontLeftMotor = frontLeft;
        m_frontRightMotor = frontRight;
        m_backLeftMotor = backLeft;
        m_backRightMotor = backRight;
    }

    public void driveFieldCentric(ChassisSpeeds speeds, Rotation2d gyroAngle) {
        m_speeds = speeds;
        m_gyroAngle = gyroAngle;
    }

    @Override
    public void periodic()
    {
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                m_speeds.vxMetersPerSecond,
                m_speeds.vyMetersPerSecond,
                m_speeds.omegaRadiansPerSecond,
                m_gyroAngle);

        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(robotSpeeds);

        //m_frontLeftMotor.set();
    }
}
