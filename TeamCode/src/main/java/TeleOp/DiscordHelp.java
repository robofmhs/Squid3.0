package TeleOp;//package org.firstinspires.ftc.teamcode.TeleOp;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous
//public class BasicAuton extends LinearOpMode {
//
//    private DcMotorEx leftFront;
//    private DcMotorEx leftRear;
//    private DcMotorEx rightFront;
//    private DcMotorEx rightRear;
//    private int count;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
//        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
//        rightRear = hardwareMap.get(DcMotorEx.class, "br");
//        rightFront = hardwareMap.get(DcMotorEx.class, "fl");
//
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            while (count==0){
//                leftFront.setPower(1);
//                leftRear.setPower(1);
//                rightRear.setPower(1);
//                rightFront.setPower(1);
//                sleep(1000);
//                leftFront.setPower(0);
//                leftRear.setPower(0);
//                rightRear.setPower(0);
//                rightFront.setPower(0);
//            }
//
//        }
//    }
//}
