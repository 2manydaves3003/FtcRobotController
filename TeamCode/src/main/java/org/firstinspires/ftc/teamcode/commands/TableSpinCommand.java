package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TableSpinnerSubsystem;

public class TableSpinCommand extends CommandBase {

    private final TableSpinnerSubsystem m_tableSpinnerSubsystem;
    private double m_power;
    private ElapsedTime m_timer;

    public TableSpinCommand(TableSpinnerSubsystem tableSpinner, double power) {
        m_tableSpinnerSubsystem = tableSpinner;
        m_power = power;
        addRequirements(m_tableSpinnerSubsystem);
        m_timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        m_tableSpinnerSubsystem.stop();
        m_timer.reset();
    }

    @Override
    public void execute() {
        m_tableSpinnerSubsystem.spin(m_power);
    }

    @Override
    public void end(boolean interrupted) {
        //if (interrupted) {
            m_tableSpinnerSubsystem.stop();
        //}
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || (m_timer.time() > 6) ;
    }

}