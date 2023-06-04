package frc.robot;

public final class Constants {
    public class DRIVETRAIN {
        // robot width (meters)
        public static final double ROBOT_WIDTH = 0.6858;
        // wheel diameter (meters)
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_PERIMETER = WHEEL_DIAMETER * Math.PI;
        // drive gear ratio
        public static final double DRIVE_GEAR_RATIO = 6.75;

        // encoder offsets (degrees)
        public static final double FL_ECODER_OFFSET = -313.506+0.5;
        public static final double FR_ECODER_OFFSET = -69.082+0.5;
        public static final double BL_ECODER_OFFSET = -45.791 + 180;
        public static final double BR_ECODER_OFFSET = -257.783;
        
        /** maximum strafe speed (meters per second) */
        public static final double MAX_LINEAR_SPEED = 5.4;
        /** maximum rotation speed (radians per second) */
        public static final double MAX_ROTATION_SPEED = Math.PI*2;
        public static final double SWERVE_SLOW_SPEED_PERCENTAGE = 0.1;
        public static final double ROTATION_SCALE_FACTOR = 0.65;
            
        // pid values
        //Note: I switched back to SDS Azimuth values, since we are using interanl FX controller
        public static final double AZIMUTH_kP = 0.2;//0.0105;//0.0115//0.0125;//0.025 //0.05//0.1 //0.01 //0.0053 sds: 0.2; rylan: 0.65
        public static final double AZIMUTH_kD = 0;//0.000265;//0.000275;//0.0003;//0.0004;//0.0005;//0.0006;//0.0006125;//0.0006125//0.000625//0.00065//0.0006;//0.00055//0.0005;//0.002//0.001//0.00075 //0.0005;//0.00025
        public static final double AZIMUTH_kF = 0.05;//0.05
        public static final double AZIMUTH_DEADBAND = 0.06;//0.1;//0.06;//0.075over slop;//0.1Over slop//0.05 under slop

        // calculated via JVN calculator
        public static final double DRIVE_kP = 0.088062; //0.04;//0.07;//0.06; //0.044057
        public static final double DRIVE_kF = 0.028998;//0.04//0.06; //0.028998

        // Factor to make odometry accurate
        public static final double SCALE_FACTOR = 0.02;

        //Ids
        public static final int FL_CANCODER_ID = 4;
        public static final int FR_CANCODER_ID = 5;
        public static final int BL_CANCODER_ID = 6;
        public static final int BR_CANCODER_ID = 7;

        public static final int FL_DRIVE_ID = 11;
        public static final int FR_DRIVE_ID = 12;
        public static final int BL_DRIVE_ID = 13;
        public static final int BR_DRIVE_ID = 14;

        public static final int FL_AZIMUTH_ID = 21;
        public static final int FR_AZIMUTH_ID = 22;
        public static final int BL_AZIMUTH_ID = 23;
        public static final int BR_AZIMUTH_ID = 24;

        public static final double _translationKp = 3.75;//3.25;//2.75;//2.5;//2.1;//2;//0.018;//0.03;//0.004 0.001
        public static final double _translationKi = 0;
        public static final double _translationKd = 0;
        public static final double _rotationKp = 6.25;//12.5;//15;//0.00005
        public static final double _rotationKi = 0;
        public static final double _rotationKd = 0;
}
}
