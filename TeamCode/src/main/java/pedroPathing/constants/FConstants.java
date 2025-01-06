package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "fl";
        FollowerConstants.leftRearMotorName = "bl";
        FollowerConstants.rightFrontMotorName = "fr";
        FollowerConstants.rightRearMotorName = "br";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 11.6;

        FollowerConstants.xMovement = 74.76397957943149;
        FollowerConstants.yMovement = 56.67278614131173;

        FollowerConstants.forwardZeroPowerAcceleration = -27.387077032024224;
        FollowerConstants.lateralZeroPowerAcceleration = -71.06676293196834;

//        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.15,0,0.01,0);
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.3,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.3,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

//        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

//        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.025,0,.000025,0.6,0);
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,.000025,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
//        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.0225,0,0.0006,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.0225,0,0.0006,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
