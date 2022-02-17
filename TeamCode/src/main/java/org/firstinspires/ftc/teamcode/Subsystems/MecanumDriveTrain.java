package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveTrain extends SubsystemBase {

    Motor m_frontLeftMotor;
    Motor m_frontRightMotor;
    Motor m_backLeftMotor;
    Motor m_backRightMotor;
    Telemetry m_telemetry;

    ChassisSpeeds m_robotCentricSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

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

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Telemetry telemetry)
    {
        m_frontLeftMotor = frontLeft;
        m_frontRightMotor = frontRight;
        m_backLeftMotor = backLeft;
        m_backRightMotor = backRight;
        m_telemetry = telemetry;
    }

    public void drive(ChassisSpeeds robotCentricSpeeds) {
        m_robotCentricSpeeds = robotCentricSpeeds;
    }

    @Override
    public void periodic()
    {
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(m_robotCentricSpeeds);

        m_telemetry.addData("FL", wheelSpeeds.frontLeftMetersPerSecond);
        m_telemetry.addData("FR", wheelSpeeds.frontRightMetersPerSecond);
        m_telemetry.addData("BL", wheelSpeeds.rearLeftMetersPerSecond);
        m_telemetry.addData("BR", wheelSpeeds.rearRightMetersPerSecond);
        m_telemetry.update();
    }
}
