package Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import Subsystems.Hang;
import Subsystems.OuttakeGripper;
import Subsystems.OuttakeWrist;
import Subsystems.PivotSlides;


@Config
public class OuttakeControl {
    public OuttakeWrist wrist;
    public OuttakeGripper gripper;
    public PivotSlides slides;
    public Hang hang;
    public static int armInterval=50;

    public OuttakeControl(OuttakeWrist wrist, OuttakeGripper gripper, PivotSlides slides, Hang hang) {
        this.wrist = wrist;
        this.gripper = gripper;
        this.slides = slides;
        this.hang = hang;
    }
    public void update(Gamepad g1, Gamepad g2) {

//
            wrist.turnWrist(0.01*-g2.right_stick_y);
//


        if(g2.y){
            slides.togglePID(true);
            slides.setSlideTarget(1050);
            if(Math.abs(slides.getSlidePos()-1050)<15){
                wrist.setWristPos(.7753);
            }
        }
        if(g2.b&&!g2.start){
            slides.togglePID(true);
            slides.setSlideTarget(1700);
//            wrist.setWristPos(.8);
            if(Math.abs(slides.getSlidePos()-1700)<25){
                gripper.setPosition(.2);
            }
        }
        if(g2.back){
            wrist.setWristPos(.8557);
            slides.togglePID(true);
            slides.setSlideTarget(0);
            if(Math.abs(slides.getSlidePos())<15){
                gripper.setPosition(.2);
            }
        }

        if(g2.x){
            wrist.setWristPos(.125);
            slides.togglePID(true);
            slides.setSlideTarget(0);
            if(Math.abs(slides.getSlidePos())<15){
                gripper.setPosition(.2);
            }
        }

        hang.setArmTarget(hang.getArmTarget()+(int)(armInterval*g2.left_stick_y));


        double slideTrig = g2.right_trigger-g2.left_trigger;
       if(slideTrig!=0) {
//           slides.setSlideTarget((int) (1.0 * slideTrig * 30) + slides.getSlideTarget());
           if(!g2.guide) {
               slides.togglePID(false);
           }
           slides.setSlideControl(slideTrig);

       }
       else {
//           slides.togglePID(true);
           slides.setSlideControl(0);

       }

        if(g2.a){
            if (gripper.OuttakeGripper.getPosition() >= 0.2 && gripper.OuttakeGripper.getPosition() < 0.2+.01) {
//                gripper.OuttakeGripper.setPosition(.7);
                gripper.setPosition(.55);

            } else if (gripper.OuttakeGripper.getPosition() >= .55) {
//                gripper.OuttakeGripper.setPosition(0);
                gripper.setPosition(0.2);

            } else {
//                gripper.OuttakeGripper.setPosition(.7);
                gripper.setPosition(.55);

            }
            while (g2.a) {
            }
        }



        wrist.update();
        gripper.update();
        slides.update();
    }
}
