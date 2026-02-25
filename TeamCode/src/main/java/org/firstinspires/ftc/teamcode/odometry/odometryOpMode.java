package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;



@TeleOp
public class odometryOpMode extends LinearOpMode {

    //Depart dans la loading zone rouge, face au human player
    double xStartingPosition = 63.25;
    double yStartingPosition = -63.15;
    double headingStartingPosition = 270.0;
    double odoOffsetX = 0;
    double odoOffsetY = 0;

    double absoluteHeadingToBlueGoal = 0;
    double relativeHeadingToBlueGoal = 0;

    GoBildaPinpointDriver odo;
    DriveToPoint nav;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    Pose2D currentTargetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Pose2D targetPoseShootFar = new Pose2D(DistanceUnit.INCH, 59, -13, AngleUnit.DEGREES, 204.25);
    Pose2D targetPoseShootClose = new Pose2D(DistanceUnit.INCH, -20, 0, AngleUnit.DEGREES, 234.16);
    Pose2D targetPoseEndgame = new Pose2D(DistanceUnit.INCH, 39, 33, AngleUnit.DEGREES, 180);

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");


        nav = new DriveToPoint(this);
        odo = this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(odoOffsetX, odoOffsetY);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();
        odo.setPosition(new Pose2D(DistanceUnit.INCH,xStartingPosition, yStartingPosition, AngleUnit.DEGREES,headingStartingPosition));

        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.a) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.dpad_left) {
                currentTargetPose = targetPoseShootFar;
                driveToTarget(currentTargetPose, 0.5);
            }

            if (gamepad1.dpad_right) {
                currentTargetPose = targetPoseShootClose;
                driveToTarget(currentTargetPose, 0.5);
            }
            if (gamepad1.dpad_up) {
                currentTargetPose = targetPoseEndgame;
                driveToTarget(currentTargetPose, 0.5);
            }

            telemetry.addData("X Position", "%5.2f", odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", "%5.2f", odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%5.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            absoluteHeadingToBlueGoal = (Math.atan((-72 - odo.getPosition().getY(DistanceUnit.INCH)) / (-72 - odo.getPosition().getX(DistanceUnit.INCH))) + Math.PI) * 180 / Math.PI;
            relativeHeadingToBlueGoal = absoluteHeadingToBlueGoal - odo.getPosition().getHeading(AngleUnit.DEGREES);
            telemetry.addData("Absolute heading to blue goal", "%5.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("Relative heading to blue goal", "%5.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
    public boolean isAtTarget(double posTolerance, double velTolerance, double angleTolerance) {
        odo.update();
        return Math.abs(odo.getPosX() - currentTargetPose.getX(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosY() - currentTargetPose.getY(DistanceUnit.MM)) < posTolerance &&
                Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - currentTargetPose.getHeading(AngleUnit.RADIANS)) < angleTolerance &&
                Math.abs(odo.getVelX()) < velTolerance &&
                Math.abs(odo.getVelY()) < velTolerance;
    }
    void driveToTarget(Pose2D targetPose, double speed) {
        odo.update();
        this.currentTargetPose = targetPose;
        nav.driveTo(odo.getPosition(), targetPose, speed, 0);
        frontLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        frontRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        backLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        backRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));
        this.telemetry.addData("TARGET Position", data2);


    }
}