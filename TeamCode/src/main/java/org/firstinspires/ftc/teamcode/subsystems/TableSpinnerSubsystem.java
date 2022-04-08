package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TableSpinnerSubsystem extends SubsystemBase
{
    HardwareMap m_hardwareMap;
    MotorEx m_spinMotor;

    public TableSpinnerSubsystem(HardwareMap hardwareMap)
    {
        m_hardwareMap = hardwareMap;
        m_spinMotor = m_hardwareMap.get(MotorEx.class, "spinner");
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
