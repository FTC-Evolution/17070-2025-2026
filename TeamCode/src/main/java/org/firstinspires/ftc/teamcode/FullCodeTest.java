package org.firstinspires.ftc.teamcode;

//😎

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;


@TeleOp
public class FullCodeTest extends LinearOpMode {
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

    double truePower;

    IMU imu;

    public Servo light = null;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

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

        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(.388);


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

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1 ----------------------------------------------------------------------------------------------------------------


        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;



        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            if (gamepad1.y){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.x){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();


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
                truePower = 900 + plusOnePower;
            }
            else if(gamepad2.right_bumper){
                highMotor.setVelocity(1200 + plusOnePower);
                lowMotor.setVelocity(1200 + plusOnePower);
                truePower = 1200 + plusOnePower;
            }
            else if (gamepad2.b){
                plusOnePower = 0;
                highMotor.setVelocity(0);
                lowMotor.setVelocity(0);
                truePower = 0.0;
            }

            if(lowMotor.getVelocity() > (truePower - 50) && lowMotor.getVelocity() < (truePower + 50)){
                if(highMotor.getVelocity() > (truePower - 50) && highMotor.getVelocity() < (truePower + 50)){
                    light.setPosition(.500);
                } else {
                    light.setPosition(0.277);
                }
            } else {
                light.setPosition(0.277);
            }

            if (gamepad2.y) {
                servo1.setPower(1);
            } else if (gamepad2.a) {
                servo1.setPower(-1);
            } else if (gamepad2.x) {
                servo1.setPower(0);
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

//code complet test