package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.Vector;
import java.util.function.DoubleSupplier;

@Config
public class TargetingLockCommand extends CommandBase {

    private final MecanumDriveSubsystem m_driveSubsystem;
    private Telemetry m_telemetry;
    private final DoubleSupplier m_leftYSupplier;
    private final DoubleSupplier m_leftXSupplier;
    private boolean m_isFieldCentric;
    private Vector2d m_targetLock;
    private PIDFController m_pidController;
    public static PIDCoefficients m_PIDCoefficients = new PIDCoefficients(2.0, 0.0, 0.0);

    public TargetingLockCommand(MecanumDriveSubsystem driveSubsystem, DoubleSupplier leftYSupplier,
                                DoubleSupplier leftXSupplier, Vector2d targetLock, Boolean isFieldCentric, Telemetry telemetry) {
        m_driveSubsystem = driveSubsystem;
        m_leftXSupplier = leftXSupplier;
        m_leftYSupplier = leftYSupplier;
        m_isFieldCentric = isFieldCentric;
        m_targetLock = targetLock;
        m_telemetry = telemetry;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_pidController = new PIDFController(m_PIDCoefficients);
        m_pidController.setInputBounds(-Math.PI, Math.PI);
        m_driveSubsystem.setPoseEstimate(new Pose2d(0.0, 0.0, 0.0));
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_driveSubsystem.getPoseEstimate();
        Vector2d targetvec = m_targetLock.minus(new Vector2d(currentPose.getX(), currentPose.getY()));
        Vector2d targetvecNorm = targetvec.div(targetvec.norm());
        Vector2d unitVec = new Vector2d(1.0, 0.0);
        double x1 = unitVec.getX();
        double y1 = unitVec.getY();
        double x2 = targetvecNorm.getX();
        double y2 = targetvecNorm.getY();
        double targetAngle = Math.atan2(x1*y2 - x2*y1, x1*x2 + y1*y2);

        m_pidController.setTargetPosition(targetAngle);
        double heading_control = m_pidController.update(currentPose.getHeading());

        m_telemetry.addData("CurrentPose", currentPose);
        m_telemetry.addData("TargetAngle", targetAngle);
        m_telemetry.addData("CurrentHeading", currentPose.getHeading());
        m_telemetry.addData("HeadingControl", heading_control);
        m_telemetry.update();

        m_driveSubsystem.drive(m_leftYSupplier.getAsDouble(),
                m_leftXSupplier.getAsDouble(),
                heading_control,
                m_isFieldCentric);
        m_driveSubsystem.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_driveSubsystem.stop();
        }
    }


}
