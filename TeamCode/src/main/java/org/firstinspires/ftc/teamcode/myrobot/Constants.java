package org.firstinspires.ftc.teamcode.myrobot;

import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveKinematics;

public final class Constants
{
    public final static double NANOSECONDSTOSECONDS = 1e-9;
    public final static double SECONDSTONANOSECONDS = 1e9;

    public static final class SWTestBot
    {
        public final static double GEARBOXRATIO = 20; // 20:1 gearbox ratio (make sure MAXRPM changes with changes to the gearbox ratios)
        public final static double TICKSPERMOTORROTATION = 24; //number of encoder ticks/counts per 1 rotation of input motor shaft
        public final static double CPR = GEARBOXRATIO * TICKSPERMOTORROTATION; //number of encoder Counts Per Revolution (CPR) of output shaft
        public final static double MAXRPM = 300; //This is max output shaft rotations/minute as per spec sheets (the default in the FTClib for this motor with 60:1 gearbox was 165).  This will need to be empirically measured

        public final static double WHEEL_RADIUS = 0.049000029; //meters
        public final static double MAXVEL = MAXRPM * 2 * Math.PI * WHEEL_RADIUS / 60.0; //include any additional gear reductions here (this does not include the motor gearbox)
        public final static double TICKSTOMETERS = WHEEL_RADIUS * 2 * Math.PI / CPR; //include any additional gear reductions here (this does not include the motor gearbox)

        public static double DRIVEBASEWIDTH = 0.254; //meters
        public static double DRIVEBASELENGTH = 0.2921; //meters

        public static double MAXOMEGA = MAXVEL / ((DRIVEBASELENGTH + DRIVEBASEWIDTH)/2.0); //This assumes all mecanum wheels are equidistant from the center of mass

        public static double DRIVEFFEEDFORWARD_KS = 0.0;
        public static double DRIVEFEEDFORWARD_KV = 1.0;
        public static double DRIVEFEEDFORWARD_KA = 0.0; //Something's weird with the Encoder acceleration estimate (it doesn't zero out when robot not moving)

        public static double DRIVEFEEDBACK_KP = 1.1;
        public static double DRIVEFEEDBACK_KI = 0.0;
        public static double DRIVEFEEDBACK_KD = 0.0;

        public static MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                new Translation2d(DRIVEBASELENGTH/2.0, DRIVEBASEWIDTH/2.0),
                new Translation2d(DRIVEBASELENGTH/2.0, -DRIVEBASEWIDTH/2.0),
                new Translation2d(-DRIVEBASELENGTH/2.0, DRIVEBASEWIDTH/2.0),
                new Translation2d(-DRIVEBASELENGTH/2.0, -DRIVEBASEWIDTH/2.0));
    }

    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        AUTO,
        VELOCITY_CONTROL_TUNE
    }
}
