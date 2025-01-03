package Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot implements Subsystem {
    public PivotSlides pivotSlides;
    public OuttakeWrist wrist;
    public OuttakeGripper OuttakeGripper;
    public IntakeGripper IntakeGripper;
    public IntakeDiffy IntakeDiffy;
    public IntakeSlides IntakeSlides;
    public Hang hang;

    public Robot(HardwareMap map) {
        pivotSlides = new PivotSlides(map);
        wrist = new OuttakeWrist(map);
        OuttakeGripper = new OuttakeGripper(map);
        IntakeGripper = new IntakeGripper(map);
        IntakeDiffy = new IntakeDiffy(map);
        IntakeSlides = new IntakeSlides(map);
        hang = new Hang(map);
    }
    @Override
    public void update() {
        pivotSlides.update();
        wrist.update();
        OuttakeGripper.update();
        IntakeGripper.update();
        IntakeDiffy.update();
        IntakeSlides.update();
        hang.update();
    }
}
