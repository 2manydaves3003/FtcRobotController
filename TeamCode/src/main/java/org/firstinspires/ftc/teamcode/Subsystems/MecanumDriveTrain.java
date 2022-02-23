package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot.Constants;

public class MecanumDriveTrain extends SubsystemBase {

    Motor m_frontLeftMotor;
    Motor m_frontRightMotor;
    Motor m_backLeftMotor;
    Motor m_backRightMotor;
    Telemetry m_telemetry;

    ChassisSpeeds m_robotCentricSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = Constants.SWTestBot.kinematics;

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Telemetry telemetry)
    {
        m_frontLeftMotor = frontLeft;
        m_frontRightMotor = frontRight;
        m_backLeftMotor = backLeft;
        m_backRightMotor = backRight;
        m_telemetry = telemetry;
    }

    public void drive(ChassisSpeeds robotCentricSpeeds) {
        m_robotCentricSpeeds = robotCentricSpeeds;
    }

    public void setRunMode(Motor.RunMode driveTrainRunMode) {
        m_frontLeftMotor.setRunMode(driveTrainRunMode);
        m_frontRightMotor.setRunMode(driveTrainRunMode);
        m_backLeftMotor.setRunMode(driveTrainRunMode);
        m_backRightMotor.setRunMode(driveTrainRunMode);
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior driveTrainZeroPowerBehavior)
    {
        //Set the Zero power behavior (check this during path following)
        m_frontLeftMotor.setZeroPowerBehavior(driveTrainZeroPowerBehavior);
        m_frontRightMotor.setZeroPowerBehavior(driveTrainZeroPowerBehavior);
        m_backLeftMotor.setZeroPowerBehavior(driveTrainZeroPowerBehavior);
        m_backRightMotor.setZeroPowerBehavior(driveTrainZeroPowerBehavior);
    }

    public void setFeedForwardCoefficients(double ks, double kv, double ka)
    {
        m_frontLeftMotor.setFeedforwardCoefficients(ks, kv, ka);
        m_frontRightMotor.setFeedforwardCoefficients(ks, kv, ka);
        m_backLeftMotor.setFeedforwardCoefficients(ks, kv, ka);
        m_backRightMotor.setFeedforwardCoefficients(ks, kv, ka);
    }

    public void setFeedBackCoefficients(double kp, double ki, double kd)
    {
        m_frontLeftMotor.setVeloCoefficients(kp, ki, kd);
        m_frontRightMotor.setVeloCoefficients(kp, ki, kd);
        m_backLeftMotor.setVeloCoefficients(kp, ki, kd);
        m_backRightMotor.setVeloCoefficients(kp, ki, kd);
    }

    @Override
    public void periodic()
    {
        //Note: this kinematic model scales the wheel speeds by 1/sqrt(2)
        MecanumDriveWheelSpeeds wheelSpeeds =
                m_kinematics.toWheelSpeeds(m_robotCentricSpeeds);

        wheelSpeeds.normalize(Constants.SWTestBot.MAXVEL); //"Normalize" is the wrong word.  It's more of a capping the requested velocities to be at/under the actual max velocity

        //Here we actually normalize the wheel speeds to convert them to a normalized power (i.e. %) between -1.0 and 1.0
        double frontLeftPower = wheelSpeeds.frontLeftMetersPerSecond / Constants.SWTestBot.MAXVEL;
        double frontRightPower = wheelSpeeds.frontRightMetersPerSecond / Constants.SWTestBot.MAXVEL;
        double rearLeftPower = wheelSpeeds.rearLeftMetersPerSecond / Constants.SWTestBot.MAXVEL;
        double rearRightPower = wheelSpeeds.rearRightMetersPerSecond / Constants.SWTestBot.MAXVEL;

//        m_telemetry.addData("input_Vx",m_robotCentricSpeeds.vxMetersPerSecond);
//        m_telemetry.addData("input_vy", m_robotCentricSpeeds.vyMetersPerSecond);
//        m_telemetry.addData("input_omega", m_robotCentricSpeeds.omegaRadiansPerSecond);
//        m_telemetry.addData("FL", wheelSpeeds.frontLeftMetersPerSecond);
//        m_telemetry.addData("FR", wheelSpeeds.frontRightMetersPerSecond);
//        m_telemetry.addData("BL", wheelSpeeds.rearLeftMetersPerSecond);
//        m_telemetry.addData("BR", wheelSpeeds.rearRightMetersPerSecond);
//        //m_telemetry.addData("MaxTicksPerSecond", m_frontLeftMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
//        //m_telemetry.addData("RPM", m_frontLeftMotor.getMaxRPM());
//        //m_telemetry.addData("CPR", m_frontLeftMotor.getCPR());
//        m_telemetry.update();

        m_frontLeftMotor.set(frontLeftPower);
        m_frontRightMotor.set(frontRightPower);
        m_backLeftMotor.set(rearLeftPower);
        m_backRightMotor.set(rearRightPower);
    }
}
