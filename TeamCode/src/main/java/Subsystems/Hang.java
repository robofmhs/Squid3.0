package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Hang implements Subsystem{
    private DcMotorEx arm, arm2;
    private int armPos;
    public static int armTarget;
    public static double kp=0.001, ki=0.00003, kd=0.000023;
    public static double kf=.001;
    private final double armTicks_in_degree = 5281.1 / 360;
    private PIDController armPIDControl;
    public Hang(HardwareMap map){
        arm = map.get(DcMotorEx.class, "arm");
        arm2 = map.get(DcMotorEx.class, "arm2");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE );
        armPIDControl = new PIDController(kp, ki, kd);
    }
    @Override
    public void update() {
        double pid2 = Math.signum(getArmPID())*Math.sqrt(Math.abs(getArmPID()));
        double armPower = getArmFF() + pid2;
        arm.setPower(armPower);
        arm2.setPower(armPower);
        armPos=arm.getCurrentPosition();
    }
    public void setArmTarget(int x){
        armTarget = x;
    }
    public int getArmTarget(){
        return armTarget;
    }
    public int getArmPos(){
        return arm.getCurrentPosition();
    }

    public double getArmPID(){
        armPIDControl.setPID(kp, ki, kd);
        double armPid = armPIDControl.calculate(armPos, armTarget);
        return armPid;
    }
    public double getArmFF() {
        double armAngle= armPos/armTicks_in_degree+90;
        double armff = Math.cos(Math.toRadians(armAngle))*kf;
        return armff;
    }

}
