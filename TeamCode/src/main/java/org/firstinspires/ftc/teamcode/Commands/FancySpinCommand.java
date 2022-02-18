package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.TableSpinner;

import java.util.concurrent.TimeUnit;

public class FancySpinCommand extends CommandBase
{
    TableSpinner m_tableSpinner;
    Timing.Timer m_timer;
    Telemetry m_telemetry;

    double startingVelocity = 0.2;
    double velocityChange = 0.8;
    double deltaTime = 3; //seconds
    double velocitySlope = velocityChange/deltaTime;
    double changeTime = 2; //seconds

    public FancySpinCommand(TableSpinner tableSpinner, Telemetry telemetry)
    {
        m_tableSpinner = tableSpinner;
        m_telemetry = telemetry;
        m_timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        addRequirements(m_tableSpinner);
    }

    @Override
    public void initialize()
    {
        m_timer.start();
    }

    @Override
    public void execute()
    {
        if(m_timer.elapsedTime()*1e-3 < changeTime)
        {
            m_tableSpinner.spin(startingVelocity);
        }
        else
        {
            double velocity = (m_timer.elapsedTime()*1e-3 - changeTime) * velocitySlope + startingVelocity;
            velocity = Math.min(velocity, 1.0);
            m_tableSpinner.spin(velocity);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_tableSpinner.stop();
    }
}
