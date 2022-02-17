package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;

import java.util.function.DoubleSupplier;

public class BasicDriveCommand extends CommandBase {
    MecanumDriveTrain m_drivetrain;
    DoubleSupplier m_xSupplier;
    DoubleSupplier m_ySupplier;
    DoubleSupplier m_omegaSupplier;

    public BasicDriveCommand(MecanumDriveTrain driveTrain,
                             DoubleSupplier xSupplier,
                             DoubleSupplier ySupplier,
                             DoubleSupplier omegaSupplier)
    {
        m_drivetrain = driveTrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_omegaSupplier = omegaSupplier;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute()
    {
        //Field-centric transformation placed here, but could go in DriveTrain
        ChassisSpeeds robotCentricSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                m_xSupplier.getAsDouble(),
                m_ySupplier.getAsDouble(),
                m_omegaSupplier.getAsDouble(),
                Rotation2d.fromDegrees(0.0));

        m_drivetrain.drive(robotCentricSpeeds);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_drivetrain.drive(new ChassisSpeeds(0.0,0.0,0.0));
    }
}
