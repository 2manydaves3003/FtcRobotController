package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalization;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class VelControlTuneCommand extends CommandBase
{
    MecanumDriveTrain m_driveTrain;
    RobotLocalization m_robotLocalization;
    Telemetry m_telemetry;

    public static PIDCoefficients VELOPID = new PIDCoefficients(0.9,0.0,0.0);
    public static double ks = 0.0;
    public static double kv = 1.2; // note: I'm setting ka to zero
    public static double maxVelMetersPerSecond = 0.4;
    public static double maxAccel = 0.5;
    public static double DISTANCEMETERS = 1.0;

    double m_flip = 1.0;

    Timing.Timer m_timer;
    Trajectory m_trajectory;
    TrajectoryConfig m_trajectoryConfig;

    Pose2d m_startPose = new Pose2d( new Translation2d(0.0,0.0), new Rotation2d(0.0));
    Pose2d m_endPose = new Pose2d( new Translation2d(DISTANCEMETERS,0.0), new Rotation2d(0.0));
    List<Translation2d> m_waypoints = Collections.<Translation2d>emptyList();
    TrajectoryConfig m_config = new TrajectoryConfig(maxVelMetersPerSecond, maxAccel);

    public VelControlTuneCommand(MecanumDriveTrain driveTrain, RobotLocalization robotLocalization, Telemetry telemetry)
    {
        m_driveTrain = driveTrain;
        m_robotLocalization = robotLocalization;
        m_telemetry = telemetry;
        addRequirements(m_driveTrain, m_robotLocalization);

        m_timer = new Timing.Timer(0, TimeUnit.NANOSECONDS);
        m_timer.start();
    }

    @Override
    public void execute()
    {
        if(m_timer.done())
        {
            if(m_flip == 1.0)
            {
                m_config = new TrajectoryConfig(maxVelMetersPerSecond, maxAccel);
                m_endPose = new Pose2d( new Translation2d(DISTANCEMETERS,0.0), new Rotation2d(0.0));
                m_trajectory = TrajectoryGenerator.generateTrajectory(m_startPose, m_waypoints, m_endPose, m_config);
                long pathTotalTime = (long)(m_trajectory.getTotalTimeSeconds() * Constants.SECONDSTONANOSECONDS);
                m_timer = new Timing.Timer(pathTotalTime, TimeUnit.NANOSECONDS);
                m_timer.start();
                m_flip = -1.0;

            }
            else
            {
                m_config = new TrajectoryConfig(maxVelMetersPerSecond, maxAccel);
                m_endPose = new Pose2d( new Translation2d(DISTANCEMETERS,0.0), new Rotation2d(0.0));
                m_trajectory = TrajectoryGenerator.generateTrajectory(m_startPose, m_waypoints, m_endPose, m_config);
                long pathTotalTime = (long)(m_trajectory.getTotalTimeSeconds() * Constants.SECONDSTONANOSECONDS);
                m_timer = new Timing.Timer(pathTotalTime, TimeUnit.NANOSECONDS);
                m_timer.start();
                m_flip = 1.0;
            }
        }

        //For some reason, the state and trajectory and total time are zero if I make the trajectory reversed, so I use a sign 'flip' to achieve the reverse direction
        Trajectory.State state = m_trajectory.sample(m_timer.elapsedTime() * Constants.NANOSECONDSTOSECONDS);
        double currentVelocity = m_flip * state.velocityMetersPerSecond;

        m_telemetry.addData("Setpoint Velocity", m_flip * state.velocityMetersPerSecond);
        MecanumDriveWheelSpeeds wheelSpeeds = m_robotLocalization.getWheelVelocities();
        m_telemetry.addData("Encoder Velocity", wheelSpeeds.frontLeftMetersPerSecond);
        //m_telemetry.update();

        m_driveTrain.drive(new ChassisSpeeds(currentVelocity, 0.0, 0.0));
        m_driveTrain.setFeedBackCoefficients(VELOPID.p, VELOPID.i, VELOPID.d);
        m_driveTrain.setFeedForwardCoefficients(ks, kv, 0.0);
    }
}
