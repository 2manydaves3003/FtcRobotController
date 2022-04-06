package org.firstinspires.ftc.teamcode.myrobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.BasicDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveStraightCommand;
import org.firstinspires.ftc.teamcode.Commands.FancySpinCommand;
import org.firstinspires.ftc.teamcode.Commands.PathFollowCommand;
import org.firstinspires.ftc.teamcode.Commands.PathFollowCommand2;
import org.firstinspires.ftc.teamcode.Commands.RRPathFollowCommand;
import org.firstinspires.ftc.teamcode.Commands.TuningCommand;
import org.firstinspires.ftc.teamcode.Commands.TuningCommandPositionPID;
import org.firstinspires.ftc.teamcode.Commands.VelControlTuneCommand;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotLocalization;
import org.firstinspires.ftc.teamcode.Subsystems.TableSpinner;

@Config
public class SWTestBot extends Robot {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;

    //DriveTrain motors
    Motor m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor;

    //Spinner motor
    MotorEx m_spinnerMotor;

    //Localization parts
    GyroEx m_gyro;
    Motor.Encoder m_encoderFrontLeft;
    Motor.Encoder m_encoderFrontRight;
    Motor.Encoder m_encoderBackLeft;
    Motor.Encoder m_encoderBackRight;

    //Subsystems
    MecanumDriveTrain m_driveTrain;
    RobotLocalization m_robotLocalization;
    TableSpinner m_tableSpinner;

    public SWTestBot(Constants.OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1)
    {
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = telemetry;
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
        initializeDriveTrain();
        initializeRobotLocalization();
        initializeTableSpinner();

        //Setup the Robot Commands/Subsystem mappings based on OpMode type
        setupOpMode(type);
    }

    private void initializeTableSpinner()
    {
        m_spinnerMotor = new MotorEx(m_hardwareMap, "spinner");
        m_tableSpinner = new TableSpinner(m_spinnerMotor);
    }

    private void initializeRobotLocalization()
    {
        m_gyro = new RevIMU(m_hardwareMap);
        m_encoderFrontLeft = m_frontLeftMotor.encoder;
        m_encoderFrontRight = m_frontRightMotor.encoder;
        m_encoderBackLeft = m_backLeftMotor.encoder;
        m_encoderBackRight = m_backRightMotor.encoder;

        m_robotLocalization = new RobotLocalization(
                m_encoderFrontLeft,
                m_encoderFrontRight,
                m_encoderBackLeft,
                m_encoderBackRight,
                m_gyro,
                m_telemetry);
    }

    private void initializeDriveTrain()
    {
        m_frontLeftMotor = new Motor(m_hardwareMap, "FL", Constants.SWTestBot.CPR, Constants.SWTestBot.MAXRPM);
        m_frontRightMotor = new Motor(m_hardwareMap, "FR", Constants.SWTestBot.CPR, Constants.SWTestBot.MAXRPM);
        m_backLeftMotor = new Motor(m_hardwareMap, "BL", Constants.SWTestBot.CPR, Constants.SWTestBot.MAXRPM);
        m_backRightMotor = new Motor(m_hardwareMap, "BR", Constants.SWTestBot.CPR, Constants.SWTestBot.MAXRPM);

        m_frontRightMotor.setInverted(true);
        m_backRightMotor.setInverted(true);

        m_driveTrain = new MecanumDriveTrain(m_frontLeftMotor,
                m_frontRightMotor,
                m_backLeftMotor,
                m_backRightMotor,
                m_telemetry,
                Constants.SWTestBot.kinematics);

        //Configure the drivetrain motors
        m_driveTrain.setRunMode(Motor.RunMode.VelocityControl);
        m_driveTrain.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_driveTrain.setFeedForwardCoefficients(Constants.SWTestBot.DRIVEFFEEDFORWARD_KS,
                Constants.SWTestBot.DRIVEFEEDFORWARD_KV,
                Constants.SWTestBot.DRIVEFEEDFORWARD_KA);
        m_driveTrain.setFeedBackCoefficients(Constants.SWTestBot.DRIVEFEEDBACK_KP,
                Constants.SWTestBot.DRIVEFEEDBACK_KI,
                Constants.SWTestBot.DRIVEFEEDBACK_KD);
    }

    private void setupOpMode(Constants.OpModeType type)
    {
        if(type == Constants.OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            setupTeleOp();
        }
        else if (type == Constants.OpModeType.AUTO)
        {
            m_telemetry.addData("Initialize", "Auton");
            setupAuton();
        }
        else if (type == Constants.OpModeType.VELOCITY_CONTROL_TUNE)
        {
            m_telemetry.addData("Initialize", "Velocity Control Tuning");
            setupVelocityControlTune();
        }
        m_telemetry.update();
    }

    private void setupTeleOp()
    {
        m_driveTrain.setDefaultCommand(new BasicDriveCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX()));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new FancySpinCommand(m_tableSpinner, m_telemetry))
                .whenReleased(new InstantCommand(m_tableSpinner::stop, m_tableSpinner));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DriveStraightCommand(m_driveTrain));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new TuningCommand(m_driveTrain, m_robotLocalization, m_telemetry));
                .whenPressed(new TuningCommandPositionPID(m_driveTrain, m_robotLocalization, m_telemetry));
//                .whenPressed(new PathFollowCommand2(m_driveTrain, m_robotLocalization, m_telemetry));
//                .whenPressed(new RRPathFollowCommand(m_driveTrain, m_robotLocalization));
    }

    private void setupAuton()
    {

    }

    private void setupVelocityControlTune()
    {
        schedule(new VelControlTuneCommand(m_driveTrain, m_robotLocalization, m_telemetry));
    }
}
