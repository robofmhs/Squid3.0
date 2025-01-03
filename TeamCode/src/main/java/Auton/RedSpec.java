package Auton;//
//
//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.util.InterpLUT;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//
//@Config
//@Autonomous
//public class RedSpec extends LinearOpMode {
//    private int count=0;
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime auton = new ElapsedTime();
//    private ElapsedTime auton2 = new ElapsedTime();
//
//    //arm stuff
//    private PIDController armController;
//
//    public static double armKp = 0.015, armKi = 0.0005, armKd =0.0005;
//    public static double armIncrement = 10, slideIncrement=10;
//    public static double armKf = 0.0001;
//    public static int armTarget = 0;
//    public DcMotorEx arm;
//    private final double armTicks_in_degree = 5281.1/360;
//    private double armAngle;
//
//    //slide stuff
//    private PIDController slideController;
//
//    public static double slideKp = 0.02, slideKi = 0.00087, slideKd =0.00075;
//    public static double slideKf = 0.0008;
//    public static int slideTarget = 0;
//    public DcMotorEx slide;
//    private final double slideTicks_in_degree = 537.7/360;
//
//    public static double gOpen = 0;
//    public static double gClose = .3;
////    public static double wristPos = .8;
//
//    private final double ticks_in_degree = 5281.1/360;
//
//    private DcMotorEx flMotor ;
//    private DcMotorEx frMotor ;
//    private DcMotorEx blMotor;
//    private DcMotorEx brMotor ;
//
//    private Servo Gripper;
//
//    private Servo Wrist;
//    private Servo WristR;
//
//
//    public void gripper(Servo servo1, double pos1,double pos2){
//        if (servo1.getPosition() >= pos1 && servo1.getPosition() < pos1+.01) {
//            servo1.setPosition(pos2);
//
//        } else if (servo1.getPosition() >= pos2) {
//            servo1.setPosition(pos1);
//        } else {
//            servo1.setPosition(pos2);
//        }
//    }
//    InterpLUT lut = new InterpLUT();
//    InterpLUT slideLimit = new InterpLUT();
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        flMotor= hardwareMap.get(DcMotorEx.class,"fl");
//        frMotor= hardwareMap.get(DcMotorEx.class,"fr");
//        blMotor= hardwareMap.get(DcMotorEx.class,"bl");
//        brMotor= hardwareMap.get(DcMotorEx.class,"br");
//        Gripper = hardwareMap.get(Servo.class,"OuttakeGripper"); // is port 2 in expansion
//        Wrist= hardwareMap.get(Servo.class,"wrist"); // is port 0 in expansion
////        WristR= hardwareMap.get(Servo.class,"wristR"); // is port 0 in expansion
//        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        Wrist.setPosition(0.675);
////        WristR.setPosition(0.35);
//        Gripper.setPosition(gClose);
//        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
////        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        slide = hardwareMap.get(DcMotorEx.class, "slide");
////        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setDirection(DcMotorSimple.Direction.REVERSE);
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
//        armController = new PIDController(armKp, armKi, armKd);
//        slideController = new PIDController(slideKp, slideKi, slideKd);
//        //lut has slide ticks as input, has output as armKf
//        lut.add(0, 0.038);
//        lut.add(500, .093);
//        lut.add(1000, .13);
//        lut.add(1500, .14);
//        lut.add(2000, 0.163);
//        lut.add(2100, 0.163);
//        lut.createLUT();
//        //slideLimit input armPos, output slideLimit;
//        slideLimit.add(0,0);
//        slideLimit.add(100,0);
//        slideLimit.add(200,0);
//        slideLimit.add(300,0);
//        slideLimit.add(400,0);
//        slideLimit.add(500,0);
//        slideLimit.add(600,0);
//        slideLimit.add(700,0);
//        slideLimit.add(800,0);
//        slideLimit.add(900,0);
//        slideLimit.add(1000,0);
//        slideLimit.add(1100,0);
//        slideLimit.add(1200,0);
//        slideLimit.add(1300,0);
//        slideLimit.add(1400,0);
//        slideLimit.add(1500,0);
//        slideLimit.add(1600,0);
//        slideLimit.add(1700,0);
//        slideLimit.add(1800,0);
//        slideLimit.add(1900,0);
//        slideLimit.add(2000,0);
//        slideLimit.add(2100,0);
//        slideLimit.add(2200,0);
//        slideLimit.add(2300,0);
//        slideLimit.add(2400,0);
//        slideLimit.add(2500,0);
//        slideLimit.add(2600,0);
//        slideLimit.add(2700,0);
//        slideLimit.add(2800,0);
//        slideLimit.add(2900,0);
//        slideLimit.add(3000,0);
//        slideLimit.add(3100,0);
//        slideLimit.add(3200,0);
//        slideLimit.add(3300,0);
//        slideLimit.add(3400,0);
//        slideLimit.add(3500,0);
//        slideLimit.add(3700,0);
//        slideLimit.add(3800,0);
//        slideLimit.add(3900,0);
//        slideLimit.add(4000,0);
//        slideLimit.createLUT();
//
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        // Wait for the game to start (driver presses START)
//        waitForStart();
//        runtime.reset();
//        auton.reset();
//        auton2.reset();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            while(count==0) {
//
////            move("forward", .3, 100);
////            Extarnal(2000,1);
//                arm.setTargetPosition(1294);
//                arm.setPower(.7);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sleep(1000);
//                slide.setTargetPosition(900);
//                slide.setPower(1);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Wrist.setPosition(.245);
//                move("forward",.3,1005);
//                sleep(1700);
//                slide.setTargetPosition(200);
//                slide.setPower(.7);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                sleep(1500);
////                arm.setTargetPosition(1100);
////                arm.setPower(1);
////                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////                sleep(1500);
////                Wrist.setPosition(.33);
////                Gripper.setPosition(gOpen);
////                sleep(500);
////                move("backward",.3,210);
////                move("rightTurn", .6,1300);
////                sleep(1000);
////                move("leftStrafe", .5,1300);
////                arm.setTargetPosition(720);
////                arm.setPower(1);
////                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                sleep(1000);
//////            move("forward", .5,400);
////                move("forward", .3,100);
////                sleep(1000);
////                move("forward", .3,190);
////            Gripper.setPosition(gClose);
////            sleep(500);
////            Wrist.setPosition(.15);
////            move("backward", .5,200);
////            move("rightTurn", .6,1300);
////            sleep(500);
////            move("leftStrafe", .5,1700);
////            sleep(1000);
////            arm.setTargetPosition(1150);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            Wrist.setPosition(.17);
////            move("forward", .5,330);
////            sleep(1000);
////            slide.setTargetPosition(1150);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(2000);
////            Gripper.setPosition(gOpen);
////            sleep(500);
////            Wrist.setPosition(.17);
////            slide.setTargetPosition(10);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            arm.setTargetPosition(1300);
////            arm.setPower(.3);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            move("backward",1,450);
////            move("rightStrafe",1,1800);
//
//
////            Wrist.setPosition(0);
////            arm.setTargetPosition(100);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            move("rightStrafe", .5,1050);
////            sleep(1000);
////            move("forward", .7,1500);
////            move("rightStrafe", .5,600);
////            move("backward", .5,3200);
////
////            arm.setTargetPosition(800);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            Wrist.setPosition(.3);
////
////            move("forward", .5,700);
////            move("rightTurn", .7,2600);
////            move("forward", .3,100);
////            sleep(1000);
////            move("forward", .3,50);
////            Gripper.setPosition(gClose);
////            sleep(500);
////            arm.setTargetPosition(1860);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            move("backward",1,200);
////            Wrist.setPosition(.60);
////            move("rightTurn", 1,2500);
////            move("leftStrafe", .5,1500);
////            move("forward",.7,700);
////            arm.setTargetPosition(1100);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(1500);
////            Gripper.setPosition(gOpen);
////            move("backward",1,900);
////            move("rightStrafe", .5,1500);
//
//
//
//
////            move("forward", .5,1500);
//
//
//
//
//
//
//
//
////            arm.setTargetPosition(1100);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(500);
////            move("leftStrafe",.5,1650);
////            sleep(2000);
////            slide.setTargetPosition(760);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(750);
////            arm.setTargetPosition(800);
////            arm.setPower(.3);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(1500);
////            Gripper.setPosition(gClose);
////            sleep(1000);
////            Wrist.setPosition(.5);
////            slide.setTargetPosition(100);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            arm.setTargetPosition(2500);
////            arm.setPower(.3);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(1700);
////            move("rightTurn",.4,1200);
////            move("backward",.3,420);
////            slide.setTargetPosition(1900);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(500);
////            Wrist.setPosition(.3);
////            sleep(500);
////            Gripper.setPosition(gOpen);
////            sleep(1500);
////            arm.setTargetPosition(2000);
////            arm.setPower(.7);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(1000);
////            slide.setTargetPosition(1);
////            slide.setPower(.7);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(2000);
////            arm.setTargetPosition(100);
////            arm.setPower(.7);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(100);
//
//
//
//
////            move("leftTurn",.3,220);
////
////            arm.setTargetPosition(2000);
////            arm.setPower(1);
////            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(500);
////            slide.setTargetPosition(2000);
////            slide.setPower(1);
////            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            sleep(1000);
////            Wrist.setPosition(.65);
////            sleep(2000);
////            move("forward",.3,500);
////            sleep(3000);
////            Gripper.setPosition(gOpen);
////            sleep(500);
//
//
////            sleep(1000);
////            slideTarget=1900;
////            sleep(500);
////            Wrist.setPosition(.8);
////            sleep(500);
////            Gripper.setPosition(gOpen);
//                count+=1;
//            }
//
//
//
//        }
//    }
//    private void Extarnal(int armTicks,int slideTicks){
//        armTarget=armTicks;
//        slideTarget=slideTicks;
//        int armPos = arm.getCurrentPosition();
//        int slidePos = Range.clip(slide.getCurrentPosition(),1,1000);
//        slideTarget= Range.clip(slideTarget,1,2000);
//        armTarget= Range.clip(armTarget,0,2200);
//        armController.setPID(armKp, armKi, armKd);
//
//        double armPid = armController.calculate(armPos, armTarget);
//
//        armAngle= armPos/armTicks_in_degree-65;
//        armKf=lut.get(slidePos);
//        double armff = Math.cos(Math.toRadians(armAngle))*armKf;
////        double ff = Math.sin(Math.toRadians(armPos/armTicks_in_degree+5)*armKf);
//        double armPower = armPid + armff;
//
//
//        slideController.setPID(slideKp, slideKi, slideKd);
//        double slidePid = slideController.calculate(slidePos, slideTarget);
//        double slideff = Math.sin(armAngle*slideKf);
//        double slidePower = slidePid + slideff;
//        slide.setPower(slidePower);
//        arm.setPower(armPower);
//    }
//    private void move(String direction, double speed,int ticks){
//        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ticks = -ticks;
//        if(direction.equals("forward")){
//            flMotor.setTargetPosition(ticks);
//            frMotor.setTargetPosition(ticks);
//            blMotor.setTargetPosition(ticks);
//            brMotor.setTargetPosition(ticks);
//        } else if (direction.equals("backward")) {
//            frMotor.setTargetPosition(-ticks);
//            flMotor.setTargetPosition(-ticks);
//            blMotor.setTargetPosition(-ticks);
//            brMotor.setTargetPosition(-ticks);
//        } else if (direction.equals("leftStrafe")) {
//            flMotor.setTargetPosition(-ticks);
//            frMotor.setTargetPosition(ticks);
//            blMotor.setTargetPosition(ticks);
//            brMotor.setTargetPosition(-ticks);
//        } else if (direction.equals("rightStrafe")) {
//            flMotor.setTargetPosition(ticks);
//            frMotor.setTargetPosition(-ticks);
//            blMotor.setTargetPosition(-ticks);
//
//            brMotor.setTargetPosition(ticks);
//        } else if (direction.equals("rightTurn")) {
//            flMotor.setTargetPosition(ticks);
//            frMotor.setTargetPosition(-ticks);
//            blMotor.setTargetPosition(ticks);
//            brMotor.setTargetPosition(-ticks);
//        } else if (direction.equals("leftTurn")) {
//            flMotor.setTargetPosition(-ticks);
//            frMotor.setTargetPosition(ticks);
//            blMotor.setTargetPosition(-ticks);
//            brMotor.setTargetPosition(ticks);
//        }
//        flMotor.setPower(speed);
//        frMotor.setPower(speed);
//        blMotor.setPower(speed);
//        brMotor.setPower(speed);
//
//        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        sleep(1000);
//
//    }
//}
