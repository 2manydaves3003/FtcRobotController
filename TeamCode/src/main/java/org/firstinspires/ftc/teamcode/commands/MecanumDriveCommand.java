package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_leftYSupplier;
    private final DoubleSupplier m_leftXSupplier;
    private final DoubleSupplier m_rightXSupplier;
    private boolean m_isFieldCentric;

    public MecanumDriveCommand(MecanumDriveSubsystem drive, DoubleSupplier leftYSupplier,
                               DoubleSupplier leftXSupplier, DoubleSupplier rightXSupplier, boolean isFieldCentric) {
        m_driveSubsystem = drive;
        m_leftXSupplier = leftXSupplier;
        m_leftYSupplier = leftYSupplier;
        m_rightXSupplier = rightXSupplier;
        m_isFieldCentric = isFieldCentric;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(m_leftYSupplier.getAsDouble(),
                m_leftXSupplier.getAsDouble(),
                m_rightXSupplier.getAsDouble(),
                m_isFieldCentric);
        m_driveSubsystem.update();
    }

}
