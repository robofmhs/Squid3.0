package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeGripper implements Subsystem{
    public Servo IntakeGripper;
    public static double gripperPos;
    public static double gOpen = .3;
    public static double gClose = .6;
    @Override
    public void update(){
        IntakeGripper.setPosition(gripperPos);
        gripperPos= IntakeGripper.getPosition();
    }
    public void setPosition(double pos) {
        gripperPos = pos;
    }
    public IntakeGripper(HardwareMap map) {
        IntakeGripper = map.get(Servo.class, "IntakeGripper");
    }
    public void toggleGripper(){
        if(gripperPos==gOpen){
            gripperPos=gClose;
        } else {
            gripperPos=gOpen;
        }
    }
}
