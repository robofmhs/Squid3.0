package Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import Subsystems.IntakeDiffy;
import Subsystems.IntakeGripper;
import Subsystems.IntakeSlides;


public class IntakeControl {
    public IntakeSlides slides;
    public IntakeDiffy diffy;
    public IntakeGripper gripper;
    double mult = .5;
    public IntakeControl(IntakeSlides slides, IntakeDiffy diffy, IntakeGripper gripper) {
        this.slides = slides;
        this.diffy = diffy;
        this.gripper = gripper;
    }

    public void update(Gamepad g1, Gamepad g2) {
        if(g1.back){
            diffy.setPosition(.7443,.5349);
            slides.setPosition(0.1678);
            gripper.setPosition(0.6);
        }
        if(g1.left_bumper){
            mult=.5;
        }
        else{

            mult=1;
        }

        if(g1.dpad_up) {
            diffy.turnDiffy(0.003*mult);
        }
        else if(g1.dpad_down) {
            diffy.turnDiffy(-0.003*mult);
        }
        else if(g1.dpad_left) {
            diffy.rotateDiffy(0.003*mult);
        }
        else if(g1.dpad_right) {
            diffy.rotateDiffy(-0.003*mult);
        }
        else {
            diffy.turnDiffy(0);
            diffy.rotateDiffy(0);
        }
        slides.changePosition(-.015*(g1.left_trigger-g1.right_trigger));
        if(g1.x){
            diffy.setPosition(.6,.6);
            slides.setPosition(1);
            gripper.setPosition(.2);
        }
        if(g1.y){
            diffy.setPosition(.567,.567);
            slides.setPosition(0);
        }

        if(g1.a){
            if (gripper.IntakeGripper.getPosition() >= .1 && gripper.IntakeGripper.getPosition() < .1+.01) {
                gripper.IntakeGripper.setPosition(.6);
                gripper.setPosition(.6);

            } else if (gripper.IntakeGripper.getPosition() >= .6) {
                gripper.IntakeGripper.setPosition(1);
                gripper.setPosition(.1);

            } else {
                gripper.IntakeGripper.setPosition(.6);
                gripper.setPosition(.6);

            }
            while (g1.a) {
            }
        }

        gripper.update();
        diffy.update();
        slides.update();
    }
}
