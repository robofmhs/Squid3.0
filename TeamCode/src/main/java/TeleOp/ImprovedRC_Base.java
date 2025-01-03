package TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controls.IntakeControl;
import org.firstinspires.ftc.teamcode.Controls.OuttakeControl;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


@TeleOp
@Config
public class ImprovedRC_Base extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    double multipier=.35;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IntakeControl intakeControl = new IntakeControl(robot.IntakeSlides, robot.IntakeDiffy, robot.IntakeGripper);
        OuttakeControl outtakeControl = new OuttakeControl(robot.wrist, robot.OuttakeGripper, robot.pivotSlides,robot.hang);
        Subsystem[] subsystems = new Subsystem[]{robot};

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.wrist.setWristPos(robot.wrist.getWristPos());
        robot.OuttakeGripper.setPosition(.55);
        robot.pivotSlides.togglePID(false);
        waitForStart();

        while(opModeIsActive()) {
            for(Subsystem system : subsystems) system.update();
            intakeControl.update(gamepad1, gamepad2);
            outtakeControl.update(gamepad1, gamepad2);
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

            if (gamepad1.right_bumper){
                multipier = .35;
            }
            else{
                multipier = 1;
            }

            leftFront.setPower(frontLeftPower*multipier);
            leftRear.setPower(backLeftPower*multipier);
            rightFront.setPower(frontRightPower*multipier);
            rightRear.setPower(backRightPower*multipier);
            telemetry.addData("slidePos",robot.pivotSlides.getSlidePos());
            telemetry.addData("wristPos",robot.wrist.getWristPos());
            telemetry.addData("hangPos",robot.hang.getArmPos());
            telemetry.addData("hangTarget",robot.hang.getArmTarget());
            telemetry.update();



        }
    }
}
