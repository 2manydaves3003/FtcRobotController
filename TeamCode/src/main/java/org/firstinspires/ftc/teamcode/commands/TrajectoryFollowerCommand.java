package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;

import java.util.List;

public class TrajectoryFollowerCommand extends CommandBase {

    private final MecanumDriveSubsystem m_driveSubsystem;
    private final Trajectory m_trajectory;

    public TrajectoryFollowerCommand(MecanumDriveSubsystem driveSubsystem, String trajectoryName) {
        m_driveSubsystem = driveSubsystem;
        m_trajectory = AssetsTrajectoryManager.load(trajectoryName);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        m_driveSubsystem.followTrajectory(m_trajectory);
    }

    @Override
    public void execute() {
        m_driveSubsystem.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !m_driveSubsystem.isBusy();
    }

}
