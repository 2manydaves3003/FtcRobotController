package org.firstinspires.ftc.teamcode.MyRobot;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;

public class SWTestBot extends Robot {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;

    //DriveTrain motors
    Motor m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor;
    MecanumDriveTrain m_driveTrain;

    public SWTestBot(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;

        initializeDriveTrain();
        initializeOpMode(type);
    }

    private void initializeDriveTrain()
    {
        m_frontLeftMotor = new Motor(m_hardwareMap, "FL");
        m_frontRightMotor = new Motor(m_hardwareMap, "FR");
        m_backLeftMotor = new Motor(m_hardwareMap, "BL");
        m_backRightMotor = new Motor(m_hardwareMap, "BR");

        m_frontRightMotor.setInverted(true);
        m_backRightMotor.setInverted(true);

        m_frontLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        m_frontRightMotor.setRunMode(Motor.RunMode.VelocityControl);
        m_backLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        m_backRightMotor.setRunMode(Motor.RunMode.VelocityControl);

        m_driveTrain = new MecanumDriveTrain(m_frontLeftMotor, m_frontRightMotor,m_backLeftMotor,m_backRightMotor);
    }

    private void initializeOpMode(OpModeType type)
    {
        if(type == OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            initializeTeleOp();
        }
        else if (type == OpModeType.AUTO)
        {
            m_telemetry.addData("Initialize", "Auton");
            initializeAuton();
        }
    }

    private void initializeTeleOp()
    {

    }

    private void initializeAuton()
    {

    }
}
