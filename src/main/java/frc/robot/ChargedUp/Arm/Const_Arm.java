package frc.robot.ChargedUp.Arm;

public class Const_Arm {
    //*! Everything is in meters */
    public static final int RotateMotorCanID = 0;
    public static final int ExtendMotorCanID = 0;
    public static final double kARM_SHOULDER_z = 0;
    public static final double kARM_SHOULDER_x = 0;
    public static final double kHAND_LENGTH = 0;
    public static final double kMAX_EXTENSION_x = .120; //.120m or 120cm (width)
    public static final double kMAX_EXTENSION_z = .198; //.198m or 198cm (Height)
    public static final double kMAX_ROTATION_SPEED = .2;
    public static final double kMAX_EXTENSION_SPEED = .2;
    public static final double kROBOT_WIDTH = 0;
    public static final double kMinAngle = 0;
    public static final double kMaxAngle = 0;
    public static final double kOffsetTo0 = 47.1;

    public class FF {
        public static final double kS = 0;
        public static final double kV = 0;

    }

    public class PID {
        public static final double kP = 3.0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public class Trajectory {
        public static final double kShoulderMaxRotSpeed = 270 * Math.PI / 180; // rad/s
        public static final double kShoulderMaxRotAcelSpeed = 180 * Math.PI / 180; // rad/s^2
    }

}
