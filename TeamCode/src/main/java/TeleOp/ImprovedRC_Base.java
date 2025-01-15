package TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Controls.IntakeControl;
import Controls.OuttakeControl;
import Subsystems.Robot;
import Subsystems.Subsystem;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp
@Config
public class ImprovedRC_Base extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    double multipier=.35;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Robot robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IntakeControl intakeControl = new IntakeControl(robot.IntakeSlides, robot.IntakeDiffy, robot.IntakeGripper);
        OuttakeControl outtakeControl = new OuttakeControl(robot.wrist, robot.OuttakeGripper, robot.pivotSlides,robot.hang);
        Subsystem[] subsystems = new Subsystem[]{robot};



        robot.wrist.setWristPos(robot.wrist.getWristPos());
        robot.OuttakeGripper.setPosition(.55);
        robot.pivotSlides.togglePID(false);
        robot.hang.setArmTarget(robot.hang.getArmPos() );
        waitForStart();
        follower.startTeleopDrive();

        while(opModeIsActive()) {
            for(Subsystem system : subsystems) system.update();
            intakeControl.update(gamepad1, gamepad2);
            outtakeControl.update(gamepad1, gamepad2);


            if (gamepad1.left_bumper){
                multipier = .35;
            }
            else{
                multipier = .9;
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*multipier, -gamepad1.left_stick_x*multipier, -gamepad1.right_stick_x*multipier, true);
            follower.update();

            telemetry.addData("pivotslidePos",robot.pivotSlides.getSlidePos());
            telemetry.addData("pivotTarget",robot.pivotSlides.getSlideTarget());
            telemetry.addData("wristPos",robot.wrist.getWristPos());

            telemetry.addData("intakeSlidePos",robot.IntakeSlides.getPosition());
            telemetry.addData("intakeDiffyPos1",robot.IntakeDiffy.getPos1());
            telemetry.addData("intakeDiffyPos2",robot.IntakeDiffy.getPos2());

            telemetry.addData("hangPos",robot.hang.getArmPos());
            telemetry.addData("hangTarget",robot.hang.getArmTarget());
            telemetry.update();



        }
    }
}
