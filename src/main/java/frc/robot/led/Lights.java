package frc.robot.led;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Lights { // creates reverse and forward lights
    public static final CANdle candle = new CANdle(41);
    public static void lights(ChassisSpeeds speed){
        if(speed.vxMetersPerSecond > 0){
            candle.setLEDs(0, 100, 0);
        }
        else if(speed.vxMetersPerSecond == 0){
            candle.setLEDs(0, 0, 100);
        }
        else{
            candle.setLEDs(100, 0, 0);
        }
    }
}
