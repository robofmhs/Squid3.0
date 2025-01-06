package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeGripper implements Subsystem{
    public Servo OuttakeGripper;
    public static double gripperPos;
    public static double gOpen = .2;
    public static double gClose = .55;
    @Override
    public void update(){
        OuttakeGripper.setPosition(gripperPos);
    }
    public void setPosition(double pos) {
        gripperPos = pos;
    }
    public OuttakeGripper(HardwareMap map) {
        OuttakeGripper = map.get(Servo.class, "OuttakeGripper");
    }
    public void toggleGripper(){
        if(gripperPos==gOpen){
            gripperPos=gClose;
        } else {
            gripperPos=gOpen;
        }
    }
}
