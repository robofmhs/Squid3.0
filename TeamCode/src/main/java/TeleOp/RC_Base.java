

package TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp
public class RC_Base extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //arm stuff
    private PIDController armController;

    public static double armKp = 0.015, armKi = 0.0005, armKd =0.0005;
    public static double armIncrement = 10, slideIncrement=10;
    public static double armKf = 0.0001;
    public static int armTarget = 0;
    public DcMotorEx arm;
    private final double armTicks_in_degree = 5281.1/360;
    private double armAngle;

    //slide stuff
    private PIDController slideController;

    public static double slideKp = 0.02, slideKi = 0.00087, slideKd =0.00075;
    public static double slideKf = 0.0008;
    public static int slideTarget = 0;
    public DcMotorEx slide;
    private final double slideTicks_in_degree = 537.7/360;

    public static double gOpen = 0;
    public static double gClose = .3;
//    public static double wristPos = .8;

    private final double ticks_in_degree = 5281.1/360;

    private DcMotorEx flMotor ;
    private DcMotorEx frMotor ;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor ;

    public static double driveSpeed = .7;

    private Servo Gripper;
    private Servo Wrist;

    private Servo IntakeGripper;
    private Servo IntakeDiffy1;
    private Servo IntakeDiffy2;



    public void gripper(Servo servo1, double pos1,double pos2){
        if (servo1.getPosition() >= pos1 && servo1.getPosition() < pos1+.01) {
            servo1.setPosition(pos2);

        } else if (servo1.getPosition() >= pos2) {
            servo1.setPosition(pos1);
        } else {
            servo1.setPosition(pos2);
        }
    }
    InterpLUT lut = new InterpLUT();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        flMotor= hardwareMap.get(DcMotorEx.class,"fl");
        frMotor= hardwareMap.get(DcMotorEx.class,"fr");
        blMotor= hardwareMap.get(DcMotorEx.class,"bl");
        brMotor= hardwareMap.get(DcMotorEx.class,"br");
        Gripper = hardwareMap.get(Servo.class,"OuttakeGripper"); // is port 2 in control
        Wrist= hardwareMap.get(Servo.class,"wrist"); // is port 0 in control



        Wrist.setPosition(0);
        Gripper.setPosition(gClose);
        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        armController = new PIDController(armKp, armKi, armKd);
        slideController = new PIDController(slideKp, slideKi, slideKd);
        //lut has slide ticks as input, has output as armKf
        lut.add(0, 0.001);
        lut.add(500, .035);
        lut.add(1000, .07);
        lut.add(1500, .095);
        lut.add(2000, 0.12);
        lut.add(2100, 0.163);
        lut.createLUT();


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                flMotor.setPower(frontLeftPower*0.35);
                blMotor.setPower(backLeftPower*0.35);
                frMotor.setPower(frontRightPower*0.35);
                brMotor.setPower(backRightPower*0.35);
            }
            else {
                flMotor.setPower(frontLeftPower*driveSpeed);
                blMotor.setPower(backLeftPower*driveSpeed);
                frMotor.setPower(frontRightPower*driveSpeed);
                brMotor.setPower(backRightPower*driveSpeed);
            }
            double wristPos=Wrist.getPosition();

            if(gamepad2.right_stick_y>0){
                Wrist.setPosition(wristPos+.005*Math.abs(gamepad2.right_stick_y));
            } else if (gamepad2.right_stick_y<0) {
                Wrist.setPosition(wristPos-.005*Math.abs(gamepad2.right_stick_y));
            }
            else{
                Wrist.setPosition(wristPos);
            }



            if(gamepad2.a){
                gripper(Gripper,gOpen,gClose);
                while (gamepad2.a){}
            }



//            armTarget = (int) (armTarget+armIncrement*-gamepad2.left_stick_y);
//
////            double slidetrigToJoy = gamepad1.left_trigger-gamepad1.right_trigger;
//            slideTarget = (int) (slideTarget+slideIncrement*slideTrigToJoy);

            int armPos = arm.getCurrentPosition();
            int slidePos = slide.getCurrentPosition();
            if(armPos>=1500) {
                 slidePos = Range.clip(slide.getCurrentPosition(), 1, 2000);
                slideTarget = Range.clip(slideTarget, 1, 2000);
            }
            else {
                 slidePos = Range.clip(slide.getCurrentPosition(), 1, 1100);
                slideTarget = Range.clip(slideTarget, 1, 1100);
            }
            armTarget= Range.clip(armTarget,0,2200);
            armAngle= armPos/armTicks_in_degree-65;

            armKf=lut.get(slidePos);
            double armff = Math.cos(Math.toRadians(armAngle))*armKf;
            double armJoy = -gamepad2.left_stick_y*1.0;

            arm.setPower(armJoy+armff);

            double slideff = Math.sin(armAngle)*slideKf;
            double slideTrigToJoy = (gamepad2.right_trigger-gamepad2.left_trigger)*1.0;
            slide.setPower(slideTrigToJoy+slideff);



            telemetry.addData("armPos",armPos);
            telemetry.addData("armTarget",armTarget);
            telemetry.addData("armff",armff);
            telemetry.addData("armPower",arm.getPower());
            telemetry.addData("slidePos",slidePos);
            telemetry.addData("slideTarget",slideTarget);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Gripper: ",  Gripper.getPosition());
            telemetry.addData("wrist", Wrist.getPosition());
            telemetry.update();
        }
    }
    public void externalPivot(){
        int armPos = arm.getCurrentPosition();
        int slidePos = slide.getCurrentPosition();
        armTarget= Range.clip(armTarget,0,2600);
        armController.setPID(armKp, armKi, armKd);
        double armPid = armController.calculate(armPos, armTarget);
        armAngle= armPos/armTicks_in_degree-65;
        armKf=lut.get(slidePos);
        double armff = Math.cos(Math.toRadians(armAngle))*armKf;
//        double ff = Math.sin(Math.toRadians(armPos/armTicks_in_degree+5)*armKf);
        double armPower = armPid + armff;
        arm.setPower(armPower);
    }
    public void externalPSlide(){
        int armPos = arm.getCurrentPosition();
        int slidePos = slide.getCurrentPosition();
        if(armPos>=1500) {
            slidePos = Range.clip(slide.getCurrentPosition(), 1, 2000);
            slideTarget = Range.clip(slideTarget, 1, 2000);
        }
        else {
            slidePos = Range.clip(slide.getCurrentPosition(), 1, 1100);
            slideTarget = Range.clip(slideTarget, 1, 1100);
        }
        armAngle= armPos/armTicks_in_degree-65;
        slideController.setPID(slideKp, slideKi, slideKd);
        double slidePid = slideController.calculate(slidePos, slideTarget);
        double slideff = Math.sin(armAngle*slideKf);
        double slidePower = slidePid + slideff;
        slide.setPower(slidePower);
    }
}
