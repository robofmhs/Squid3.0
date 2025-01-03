package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeDiffy implements Subsystem{
    private Servo IntakeDiffy1;
    private Servo IntakeDiffy2;
    public static double diffyPos1;
    public static double diffyPos2;
    @Override
    public void update(){
        IntakeDiffy1.setPosition(diffyPos1);
        IntakeDiffy2.setPosition(diffyPos2);
        diffyPos1 = IntakeDiffy1.getPosition();
        diffyPos2 = IntakeDiffy2.getPosition();
    }
    public void setPosition(double pos1,double pos2){
        diffyPos1=pos1;
        diffyPos2=pos2;
    }
    public double getPos1(){
        return diffyPos1;
    }
    public double getPos2() {
        return diffyPos2;
    }
    public IntakeDiffy(HardwareMap map) {
        IntakeDiffy1 = map.get(Servo.class, "IntakeDiffy1");
        IntakeDiffy2 = map.get(Servo.class, "IntakeDiffy2");
        IntakeDiffy2.setDirection(Servo.Direction.REVERSE);
        diffyPos1=.5;
        diffyPos2=.5;
        IntakeDiffy1.setPosition(diffyPos1);
        IntakeDiffy2.setPosition(diffyPos2);

    }
    public void turnDiffy(double x){
        diffyPos1+=x;
        diffyPos2+=x;
    }
    public void rotateDiffy(double x){
        diffyPos1-=x;
        diffyPos2+=x;
    }
}
