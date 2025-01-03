package Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotContstants {
    public static double OuttakeGripperOpen = 0.3;
    public static double OuttakeGripperClose = 0.6;

    public static double IntakeGripperOpen = 0.3;
    public static double IntakeGripperClose = 0.6;

    public static double IntakeSlideRetracted = 0.3;
    public static double IntakeSlideExtended = 0.6;
    public static double IntakeSlidesTransfer =  0.0;

    public static double OuttakeWristIntakeSpec = 0.3;
    public static double OuttakeWristDepositSpec = 0.6;
    public static double OuttakeWristTransfer =0.0 ;


    public static int OuttakeSlideIntakeSpec = 500;
    public static int OuttakeSlideDepositSpec = 500;
    public static int OuttakeSlidesTransfer =0 ;

    public static double IntakeDiffyPickUp= 0.0;
    public static double IntakeDiffyStraight = 0.0;
    public static double IntakeDiffyTransfer = 0.0;


}
