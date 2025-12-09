package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NewGrosBot extends LinearOpMode {
    double plusOnePower = 0.0;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    CRServo servo1;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    //Player 2
    double plusPower = 0.0;
    int lastInput = 0;
    int lastInput2 = 0;
    int lastInput3 = 0;
    int lastInput4 = 0;

    IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx highMotor = (DcMotorEx) hardwareMap.dcMotor.get("highMotor");
        DcMotorEx lowMotor = (DcMotorEx) hardwareMap.dcMotor.get("lowMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor elevator = hardwareMap.dcMotor.get("elevatorMotor");


        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        servo1 = hardwareMap.get(CRServo.class, "servo");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        lowMotor.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            if (gamepad2.dpad_up){
                if (lastInput == 0){
                    plusOnePower = plusOnePower + 50;
                }
                lastInput = 1;
            }
            else {
                lastInput = 0;
            }

            if (gamepad2.dpad_down){
                if (lastInput2 == 0){
                    plusOnePower = plusOnePower - 50;
                }
                lastInput2 = 1;
            }
            else {
                lastInput2 = 0;
            }

            if (gamepad2.dpad_right){
                if (lastInput3 == 0){
                    plusOnePower = plusOnePower + 10;
                }
                lastInput3 = 1;
            }
            else {
                lastInput3 = 0;
            }

            if (gamepad2.dpad_left){
                if (lastInput4 == 0){
                    plusOnePower = plusOnePower - 10;
                }
                lastInput4 = 1;
            }
            else {
                lastInput4 = 0;
            }

            if(gamepad2.left_bumper){
                highMotor.setVelocity(900 + plusOnePower);
                lowMotor.setVelocity(900 + plusOnePower);
            }
            else if(gamepad2.right_bumper){
                highMotor.setVelocity(1200 + plusOnePower);
                lowMotor.setVelocity(1200 + plusOnePower);
            }
            else if (gamepad2.b){
                plusOnePower = 0;
                highMotor.setVelocity(0);
                lowMotor.setVelocity(0);
            }

            if (gamepad2.y) {
                servo1.setPower(1);
            } else if (gamepad2.a) {
                servo1.setPower(-1);
            } else if (gamepad2.x) {
                servo1.setPower(0);//fuck git
            }

            if (gamepad2.right_trigger > 0.75) {
                intakeMotor.setPower(0.5);
            } else if (gamepad2.left_trigger > 0.75) {
                intakeMotor.setPower(0);
            }

            if (gamepad1.right_bumper) {
                elevator.setPower(0.5);
            } else if (gamepad1.left_bumper) {
                elevator.setPower(0);
            }

            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                imu.resetYaw();
            }
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            if (gamepad1.left_bumper) {
                // This calculates the power needed for each wheel based on the amount of forward,
                // strafe right, and rotate
                double frontLeftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + -gamepad1.right_stick_x;
                double frontRightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - -gamepad1.right_stick_x;
                double backRightPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - -gamepad1.right_stick_x;
                double backLeftPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + -gamepad1.right_stick_x;

                double maxPower = 1.0;
                double maxSpeed = 1.0;  // make this slower for outreaches

                // This is needed to make sure we don't pass > 1.0 to any wheel
                // It allows us to keep all of the motors in proportion to what they should
                // be and not get clipped
                maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));
                maxPower = Math.max(maxPower, Math.abs(backLeftPower));

                // We multiply by maxSpeed so that it can be set lower for outreaches
                // When a young child is driving the robot, we may not want to allow full
                // speed.
                frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
                frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
                backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
                backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
            } else {
                // First, convert direction being asked to drive to polar coordinates
                double theta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                // Second, rotate angle by the angle the robot is pointing
                theta = AngleUnit.normalizeRadians(theta -
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

                // Third, convert back to cartesian
                double newForward = r * Math.sin(theta);
                double newRight = r * Math.cos(theta);

                // Finally, call the drive method with robot relative forward and right amounts

                // This calculates the power needed for each wheel based on the amount of forward,
                // strafe right, and rotate
                double frontLeftPower = newForward + newRight + -gamepad1.right_stick_x;
                double frontRightPower = newForward - newRight - -gamepad1.right_stick_x;
                double backRightPower = newForward + newRight - -gamepad1.right_stick_x;
                double backLeftPower = newForward - newRight + -gamepad1.right_stick_x;

                double maxPower = 1.0;
                double maxSpeed = 1.0;  // make this slower for outreaches

                // This is needed to make sure we don't pass > 1.0 to any wheel
                // It allows us to keep all of the motors in proportion to what they should
                // be and not get clipped
                maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));
                maxPower = Math.max(maxPower, Math.abs(backLeftPower));

                // We multiply by maxSpeed so that it can be set lower for outreaches
                // When a young child is driving the robot, we may not want to allow full
                // speed.
                frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
                frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
                backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
                backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));

            }
        }

    }
}