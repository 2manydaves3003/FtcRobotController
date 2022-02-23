package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalization;

import java.util.function.DoubleSupplier;

public class BasicOdometryCommand extends CommandBase
{
    RobotLocalization m_robotLocalization;

    public BasicOdometryCommand(RobotLocalization robotLocalization)
    {
        m_robotLocalization = robotLocalization;
        addRequirements(m_robotLocalization);
    }

    @Override
    public void execute()
    {
    }
}
