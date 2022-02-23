package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot.Constants;

import java.util.concurrent.TimeUnit;

public class RobotLocalization extends SubsystemBase
{
    Motor.Encoder m_frontLeftEncoder;
    Motor.Encoder m_frontRightEncoder;
    Motor.Encoder m_backLeftEncoder;
    Motor.Encoder m_backRightEncoder;
    GyroEx m_gyro;
    MecanumDriveWheelSpeeds m_wheelSpeeds;

    MecanumDriveOdometry m_odometry;
    Telemetry m_telemetry;

    Timing.Timer m_timer;

    public RobotLocalization(Motor.Encoder frontLeftEncoder,
                             Motor.Encoder frontRightEncoder,
                             Motor.Encoder backLeftEncoder,
                             Motor.Encoder backRightEncoder,
                             GyroEx gyro,
                             Telemetry telemetry)
    {
        m_frontLeftEncoder = frontLeftEncoder;
        m_frontRightEncoder = frontRightEncoder;
        m_backLeftEncoder = backLeftEncoder;
        m_backRightEncoder = backRightEncoder;
        m_gyro = gyro;
        m_telemetry = telemetry;

        m_wheelSpeeds = new MecanumDriveWheelSpeeds();

        m_odometry = new MecanumDriveOdometry(Constants.SWTestBot.kinematics, m_gyro.getRotation2d(),
                new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0)));

        m_timer = new Timing.Timer((long)(2000 * Constants.SECONDSTONANOSECONDS), TimeUnit.NANOSECONDS );
        m_timer.start();
    }

    public MecanumDriveWheelSpeeds getWheelVelocities()
    {
        return m_wheelSpeeds;
    }

    public Pose2d getRobotPose()
    {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void periodic()
    {
        double frontLeftVelocity = m_frontLeftEncoder.getCorrectedVelocity() * Constants.SWTestBot.TICKSTOMETERS;
        double frontRightVelocity = m_frontRightEncoder.getCorrectedVelocity() * Constants.SWTestBot.TICKSTOMETERS;
        double rearLeftVelocity = m_backLeftEncoder.getCorrectedVelocity() * Constants.SWTestBot.TICKSTOMETERS;
        double rearRightVelocity = m_backRightEncoder.getCorrectedVelocity() * Constants.SWTestBot.TICKSTOMETERS;
        m_wheelSpeeds = new MecanumDriveWheelSpeeds(frontLeftVelocity,
                frontRightVelocity,
                rearLeftVelocity,
                rearRightVelocity);

        m_odometry.updateWithTime(m_timer.elapsedTime() * Constants.NANOSECONDSTOSECONDS,
                m_gyro.getRotation2d(),
                m_wheelSpeeds);

        m_telemetry.addData("X",m_odometry.getPoseMeters().getX());
        m_telemetry.addData("Y",m_odometry.getPoseMeters().getY());
        m_telemetry.addData("angle",m_odometry.getPoseMeters().getHeading());
        m_telemetry.update();
    }
}
