package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private SampleMecanumDrive m_drive;
    private HardwareMap m_hardwareMap;
    private Telemetry m_telemetry;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_drive = new SampleMecanumDrive(hardwareMap, telemetry);
    }

    public void setMode(DcMotor.RunMode mode) {
        m_drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        m_drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        m_drive.setPoseEstimate(pose);
    }

    public void update() {
        m_drive.update();
    }

    public void updatePoseEstimate() {
        m_drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX, boolean isFieldCentric) {
        Pose2d poseEstimate = getPoseEstimate();

//        m_telemetry.addData("poseX", poseEstimate.getX());
//        m_telemetry.addData("poseY", poseEstimate.getY());
//        m_telemetry.addData("Heading", poseEstimate.getHeading());
//        m_telemetry.update();

        Vector2d input = new Vector2d(leftY, leftX).rotated(
                isFieldCentric ? -poseEstimate.getHeading() : 0
        );

        m_drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        rightX
                )
        );
    }

    public void setDrivePower(Pose2d drivePower) {
        m_drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return m_drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return m_drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return m_drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return m_drive.trajectoryBuilder(startPose, startHeading);
    }

    public void followTrajectory(Trajectory trajectory) {
        m_drive.followTrajectoryAsync(trajectory);
    }

    public boolean isBusy() {
        return m_drive.isBusy();
    }

    public void turn(double radians) {
        m_drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return m_drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0, true);
    }

    public Pose2d getPoseVelocity() {
        return m_drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return m_drive.getLocalizer();
    }

}
