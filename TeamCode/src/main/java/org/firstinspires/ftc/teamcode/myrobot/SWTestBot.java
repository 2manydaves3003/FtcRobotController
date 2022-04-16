package org.firstinspires.ftc.teamcode.myrobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TableSpinCommand;
import org.firstinspires.ftc.teamcode.commands.TargetingLockCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TableSpinnerSubsystem;

@Config
public class SWTestBot extends Robot {
    //Basic hardware structures
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;

    //Subsystems
    MecanumDriveSubsystem m_driveTrain;
    TableSpinnerSubsystem m_tableSpinner;

    public SWTestBot(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1)
    {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
        m_driveTrain = new MecanumDriveSubsystem(m_hardwareMap, m_telemetry);
        m_tableSpinner = new TableSpinnerSubsystem(m_hardwareMap);

        //Setup the Robot Commands/Subsystem mappings based on OpMode type
        setupOpMode(type);
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

        m_telemetry.update();
    }

    private void setupTeleOp()
    {
        m_driveTrain.setDefaultCommand(new TargetingLockCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                new Vector2d(20.0,0.0),
                true,
                m_telemetry
        ));
//        m_driveTrain.setDefaultCommand(new MecanumDriveCommand(m_driveTrain,
//                ()->m_gamePad1.getLeftY(),
//                ()->-m_gamePad1.getLeftX(),
//                ()->-m_gamePad1.getRightX(),
//                true));
        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new TrajectoryFollowerCommand(m_driveTrain, "test3"));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ParallelCommandGroup(
                        new TrajectoryFollowerCommand(m_driveTrain, "test3"),
                        new TableSpinCommand(m_tableSpinner, 0.75)));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new InstantCommand(() -> {m_tableSpinner.spin(0.75);}))
                .whenReleased(new InstantCommand(m_tableSpinner::stop, m_tableSpinner));
        /*
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

         */
    }

    private void setupAuton()
    {

    }

}
