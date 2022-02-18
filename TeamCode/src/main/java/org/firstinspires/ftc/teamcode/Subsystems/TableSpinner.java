package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class TableSpinner extends SubsystemBase
{
    MotorEx m_spinMotor;

    public TableSpinner(MotorEx motor)
    {
        m_spinMotor = motor;
    }

    public void spin(double power)
    {
        m_spinMotor.set(power);
    }

    public void stop()
    {
        m_spinMotor.set(0.0);
    }

    @Override
    public void periodic()
    {

    }
}
