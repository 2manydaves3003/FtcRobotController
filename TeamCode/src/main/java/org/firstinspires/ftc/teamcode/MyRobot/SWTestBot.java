package org.firstinspires.ftc.teamcode.MyRobot;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.BasicDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.FancySpinCommand;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.TableSpinner;

public class SWTestBot extends Robot {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;

    //DriveTrain motors
    Motor m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor;

    //Spinner motor
    MotorEx m_spinnerMotor;

    //Subsystems
    MecanumDriveTrain m_driveTrain;
    TableSpinner m_tableSpinner;

    public SWTestBot(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_gamePad1 = new GamepadEx(gamePad1);
        
        initializeDriveTrain();
        initializeTableSpinner();
        initializeOpMode(type);
    }

    private void initializeTableSpinner() {
        m_spinnerMotor = new MotorEx(m_hardwareMap, "spinner");

        m_tableSpinner = new TableSpinner(m_spinnerMotor);
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

        m_driveTrain = new MecanumDriveTrain(m_frontLeftMotor,
                m_frontRightMotor,
                m_backLeftMotor,
                m_backRightMotor,
                m_telemetry);
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
        m_driveTrain.setDefaultCommand(new BasicDriveCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->m_gamePad1.getLeftX(),
                ()->m_gamePad1.getRightX()));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new FancySpinCommand(m_tableSpinner, m_telemetry))
                .whenReleased(new InstantCommand(m_tableSpinner::stop, m_tableSpinner));
    }

    private void initializeAuton()
    {

    }
}
