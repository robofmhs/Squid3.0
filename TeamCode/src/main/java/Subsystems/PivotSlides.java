package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class PivotSlides implements Subsystem{
    private DcMotorEx slide;
    private PIDController slideController;
    public static double slideKp = 0.015, slideKi = 0.000, slideKd =0.000366;
    public static double slideKf = 0.0008;
    private int slidePos;
    public static int slideTarget;
    private double slidePower;
    private boolean isPID =true;
    private double slidePid;
    private double slideControl;

    public PivotSlides(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slideController = new PIDController(slideKp,slideKi,slideKd);
        slideTarget=slide.getCurrentPosition();
    }

    @Override
    public void update() {
        if(isPID) {
            slidePower = getSlideFF() + getSlidePID();
            slide.setPower(slidePower);
        }
        else{
            slidePower = getSlideFF() + slideControl;
            slide.setPower(slidePower);
        }
        slidePos= Range.clip(slide.getCurrentPosition(),1,2000);
        slideTarget= Range.clip(slideTarget,1,2000);
    }
    public void setSlideControl(double x){slideControl=x;}
    public void togglePID(boolean x){
        isPID=x;
        slideTarget=getSlidePos();
    }
    public void setSlideTarget(int x){
        slideTarget=x;
    }
    public int getSlideTarget(){
        return slideTarget;
    }
    public int getSlidePos() {
        return slide.getCurrentPosition();
    }
    public double getSlidePID() {
        slideController.setPID(slideKp, slideKi, slideKd);
        slidePid = slideController.calculate(slidePos, slideTarget);
        return slidePid;
    }
    public double getSlideFF() {
        double slideff = slideKf;
        return slideff;
    }


}
