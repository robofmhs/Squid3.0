package Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class ResetSlideEncoder extends OpMode {
    DcMotor slide;
    @Override
    public void init() {
        slide = hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void loop() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Slide Encoder", slide.getCurrentPosition());
    }
}
