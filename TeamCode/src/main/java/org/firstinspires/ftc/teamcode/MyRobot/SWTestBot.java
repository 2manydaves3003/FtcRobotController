package org.firstinspires.ftc.teamcode.MyRobot;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SWTestBot extends Robot {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;

    public SWTestBot(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;

        if(type == OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            m_telemetry.update();
            initializeTeleOp();
        }
    }

    private void initializeTeleOp() {

    }
}
