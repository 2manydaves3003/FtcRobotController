package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class TurnCommand extends CommandBase {

    private final MecanumDriveSubsystem m_driveSubsystem;
    private final double m_angleRadians;

    public TurnCommand(MecanumDriveSubsystem drive, double angleRadians) {
        m_driveSubsystem = drive;
        m_angleRadians = angleRadians;
        
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_driveSubsystem.turn(m_angleRadians);
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
