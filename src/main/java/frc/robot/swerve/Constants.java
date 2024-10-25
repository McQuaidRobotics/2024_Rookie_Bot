package frc.robot.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public static final class kSwerve {
        public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double TRACK_WIDTH = 0.551942;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double DRIVEBASE_RADIUS = 0.39;
        public static final double DRIVEBASE_CIRCUMFERENCE = DRIVEBASE_RADIUS * (Math.PI * 2);
        public static final double ANGLE_GEAR_RATIO = 21.428571;
        public static final double DRIVE_GEAR_RATIO = 5.3571428;
        public static final double MAX_DRIVE_VELOCITY = 5.47718;
        public static final double SLIP_CURRENT = 65.0;
        public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        
    }
}
