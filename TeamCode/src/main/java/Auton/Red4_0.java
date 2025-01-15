package Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Red4+0", group = "Examples")
public class Red4_0 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private DcMotorEx outSlide;
    private Servo intakeGripper;
    private Servo outtakeGripper;
    private Servo wrist;
    private Servo intakeSlide;
    private Servo diffy1;
    private Servo diffy2;


    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7, 59, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePreLoadPose = new Pose(40.6, 67, Math.toRadians(0));

    private final Pose pickup1Control = new Pose(20, 59, Math.toRadians(0));



    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(33, 26.75, Math.toRadians(345));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(33, 17.5, Math.toRadians(345));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(44, 15.75, Math.toRadians(270));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose dropoffPose = new Pose(20, 20, Math.toRadians(180));

    private final Pose intakePosePause = new Pose(18, 26, Math.toRadians(0));

    private final Pose intakePose = new Pose(14.5, 26, Math.toRadians(0));

    private final Pose score1Pose = new Pose(43, 69, Math.toRadians(0));

    private final Pose score2Pose = new Pose(43, 73, Math.toRadians(0));

    private final Pose score3Pose = new Pose(43, 76, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(10, 22, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1,dropPickup1,intakePickup, grabPickup2,dropPickup2, grabPickup3,dropPickup3,intakePickup1,intakePickup2,intakePickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreLoadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreLoadPose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreLoadPose),new Point(pickup1Control), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePreLoadPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        dropPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(dropoffPose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), dropoffPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(dropoffPose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        dropPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(dropoffPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), dropoffPose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(dropoffPose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        dropPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(dropoffPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), dropoffPose.getHeading())
                .build();
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose), new Point(intakePosePause)))
                .setLinearHeadingInterpolation(dropoffPose.getHeading(), intakePosePause.getHeading())
                .build();
        intakePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePosePause), new Point(intakePose)))
                .setLinearHeadingInterpolation(intakePosePause.getHeading(), intakePose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(intakePose.getHeading(), score1Pose.getHeading())
                .build();
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(intakePosePause)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), intakePosePause.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(intakePose.getHeading(), score2Pose.getHeading())
                .build();
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(intakePosePause)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), intakePosePause.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakePose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(intakePose.getHeading(), score3Pose.getHeading())
                .build();


        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(score3Pose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                outSlide.setTargetPosition(1050);
                outSlide.setPower(1);
                outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(.7753);
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */


                if(Math.abs(follower.getPose().getX()-scorePreLoadPose.getX())<1&&Math.abs(follower.getPose().getY()-scorePreLoadPose.getY())<1
                        &&Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                    ) {
                    /* Score Preload */
                    outSlide.setTargetPosition(1670);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                        &&pathTimer.getElapsedTimeSeconds()>.75
                ) {
                    /* Score Preload */
                    outtakeGripper.setPosition(.2);
                    outSlide.setTargetPosition(200);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(.5);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(Math.abs(follower.getPose().getX()-pickup1Pose.getX())<1&&Math.abs(follower.getPose().getY()-pickup1Pose.getY())<1
                       ) {
                    /* Grab Sample */
                    intakeSlide.setPosition(.2653);
                    diffy1.setPosition(.504);//.489
                    diffy2.setPosition(.530);//.515

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(dropPickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > .7
                ) {
                    /* Score Preload */
                    diffy1.setPosition(.479);//.489
                    diffy2.setPosition(.505);//.515
                    intakeGripper.setPosition(.6);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

//                    follower.followPath(dropPickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds()>.6){
                    follower.followPath(dropPickup1,true);
                    setPathState(6);
                }
                break;

            case 6:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if( Math.abs(follower.getPose().getX()-dropoffPose.getX())<1&&Math.abs(follower.getPose().getY()-dropoffPose.getY())<1) {
                    /* Score Sample */
                    diffy1.setPosition(.509);//.489
                    diffy2.setPosition(.535);//.515
                    intakeGripper.setPosition(.2);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(Math.abs(follower.getPose().getX()-pickup2Pose.getX())<1&&Math.abs(follower.getPose().getY()-pickup2Pose.getY())<1) {
                    /* Grab Sample */
                    intakeSlide.setPosition(.2623);
                    diffy1.setPosition(.509);//.489
                    diffy2.setPosition(.535);//.515

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(dropPickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 1
                ) {
                    /* Score Preload */
                    diffy1.setPosition(.479);//.489
                    diffy2.setPosition(.505);//.515
                    intakeGripper.setPosition(.6);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(dropPickup2,true);
                    setPathState(805);
                }
                break;
            case 805:
                if (pathTimer.getElapsedTimeSeconds()>.6){
                    follower.followPath(dropPickup2,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(Math.abs(follower.getPose().getX()-dropoffPose.getX())<1&&Math.abs(follower.getPose().getY()-dropoffPose.getY())<1) {
                    /* Score Sample */
                    intakeGripper.setPosition(.2);
                    intakeSlide.setPosition(.13);
                    diffy1.setPosition(.5505);//.489
                    diffy2.setPosition(.4455);//.515

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds() > .8){
                    follower.followPath(grabPickup3,true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */

                if( Math.abs(follower.getPose().getX()-pickup3Pose.getX())<1&&Math.abs(follower.getPose().getY()-pickup3Pose.getY())<1
                        &&pathTimer.getElapsedTimeSeconds()>1.6
                ) {
                    /* Grab Sample */
                    diffy1.setPosition(.5505);//.489
                    diffy2.setPosition(.4455);//.515
                    intakeGripper.setPosition(.6);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(dropPickup3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds()>.8){
                    follower.followPath(dropPickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(Math.abs(follower.getPose().getX()-dropoffPose.getX())<1&&Math.abs(follower.getPose().getY()-dropoffPose.getY())<1) {
                    /* Score Sample */
                    intakeGripper.setPosition(.2);
                    intakeSlide.setPosition(.1);
                    diffy1.setPosition(.6);//.489
                    diffy2.setPosition(.6);//.515
                    outSlide.setTargetPosition(0);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(.125);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(intakePickup1,true);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(Math.abs(follower.getPose().getX()-intakePosePause.getX())<1&&Math.abs(follower.getPose().getY()-intakePosePause.getY())<1
                ) {
                    /* Score Sample */
                    diffy1.setPosition(.7243);//.489 .7443,.5349
                    diffy2.setPosition(.5149);//.515
                    wrist.setPosition(.125);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(intakePickup,true);
                    setPathState(145);
                }
                break;
            case 145:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    follower.setMaxPower(.5);
                    follower.followPath(intakePickup,true);
                    setPathState(105);
                }
                break;
            case 105:
                if(pathTimer.getElapsedTimeSeconds()>3){
                    outtakeGripper.setPosition(.55);
                    follower.setMaxPower(1);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds()>.9){
//                    outtakeGripper.setPosition(.55);
                    outSlide.setTargetPosition(1050);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(.7753);
                    follower.followPath(scorePickup1,true);
                    setPathState(17);
                }
                break;
//            case 16:
//                if(pathTimer.getElapsedTimeSeconds()>4){
//                    outSlide.setTargetPosition(1050);
//                    outSlide.setPower(1);
//                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    wrist.setPosition(.7753);
//                    setPathState(17);
//                }
            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(Math.abs(follower.getPose().getX()-score1Pose.getX())<1&&Math.abs(follower.getPose().getY()-score1Pose.getY())<1
                        &&Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                ) {
                    /* Score Sample */
                    outSlide.setTargetPosition(1700);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    setPathState(18);
                }
                break;
            case 18:
                if (Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())>15
                &&pathTimer.getElapsedTimeSeconds()>1.5
                ){
                    outtakeGripper.setPosition(.2);
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds()>.9){
                    outtakeGripper.setPosition(.2);
                    wrist.setPosition(.125);
                    outSlide.setTargetPosition(0);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(intakePickup2);
                    setPathState(20);
                }
                break;
            case 20:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(Math.abs(follower.getPose().getX()-intakePosePause.getX())<1&&Math.abs(follower.getPose().getY()-intakePosePause.getY())<1
                        &&pathTimer.getElapsedTimeSeconds()>2
                ) {
                    /* Score Sample */
                    wrist.setPosition(.125);
                    follower.setMaxPower(.5);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(intakePickup,true);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds()>2.6){
                    follower.setMaxPower(1);
                    outtakeGripper.setPosition(.55);
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTimeSeconds()>2.6){
                    wrist.setPosition(.7753);
                    outSlide.setTargetPosition(1050);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(scorePickup2,true);
                    setPathState(23);
                }
                break;
            case 23:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(Math.abs(follower.getPose().getX()-score2Pose.getX())<1&&Math.abs(follower.getPose().getY()-score2Pose.getY())<1
                        &&Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                ) {
                    /* Score Sample */
                    outSlide.setTargetPosition(1700);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(intakePickup2,true);
                    setPathState(24);
                }
                break;
            case 24:
                if (Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())>15){
                    outtakeGripper.setPosition(.2);
                    follower.followPath(intakePickup3,true);
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.getElapsedTimeSeconds()>2.6){
                    wrist.setPosition(0.125);
                    outSlide.setTargetPosition(0);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(26);
                }
                break;
            case 26:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(Math.abs(follower.getPose().getX()-intakePosePause.getX())<1&&Math.abs(follower.getPose().getY()-intakePosePause.getY())<1
                        &&Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                        &&pathTimer.getElapsedTimeSeconds()>3
                ) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.setMaxPower(.5);
                    follower.followPath(intakePickup,true);
                    setPathState(27);
                }
                break;
            case 27:
                if(pathTimer.getElapsedTimeSeconds()>2.6){
                    outtakeGripper.setPosition(.55);
                    follower.setMaxPower(1);
                    setPathState(28);
                }
                break;
            case 28:
                if(pathTimer.getElapsedTimeSeconds()>2.6){
                    wrist.setPosition(.7753);
                    outSlide.setTargetPosition(1050);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(scorePickup3,true);
                    setPathState(29);
                }
                break;
            case 29:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(Math.abs(follower.getPose().getX()-score3Pose.getX())<1&&Math.abs(follower.getPose().getY()-score3Pose.getY())<1
                        &&Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15
                ) {
                    /* Score Sample */
                    outSlide.setTargetPosition(1700);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(intakePickup2,true);
                    setPathState(30);
                }
                break;

            case 30:
                if (Math.abs(outSlide.getCurrentPosition()-outSlide.getTargetPosition())<15){
                    outtakeGripper.setPosition(.2);
                    wrist.setPosition(0.5);
                    outSlide.setTargetPosition(200);
                    outSlide.setPower(1);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    follower.followPath(park,true);
                    setPathState(31);
                }
                break;
            case 31:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(Math.abs(follower.getPose().getX()-parkPose.getX())<1&&Math.abs(follower.getPose().getY()-parkPose.getY())<1)            {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("slidePos", outSlide.getCurrentPosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        outSlide = hardwareMap.get(DcMotorEx.class, "slide");
        outSlide.setDirection(DcMotorSimple.Direction.REVERSE);
         intakeGripper = hardwareMap.get(Servo.class, "IntakeGripper");
         outtakeGripper = hardwareMap.get(Servo.class, "OuttakeGripper");
       wrist = hardwareMap.get(Servo.class, "wrist");
        intakeSlide = hardwareMap.get(Servo.class, "IntakeSlide1");
        diffy1 = hardwareMap.get(Servo.class, "IntakeDiffy1");
         diffy2 = hardwareMap.get(Servo.class, "IntakeDiffy2");
        diffy2.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(.7753);
        outtakeGripper.setPosition(.55);
        intakeGripper.setPosition(.2);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
