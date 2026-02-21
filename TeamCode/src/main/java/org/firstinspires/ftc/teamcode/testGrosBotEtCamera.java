package org.firstinspires.ftc.teamcode;

//😎
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;


@TeleOp
public class testGrosBotEtCamera extends LinearOpMode {
    double plusOnePower = 0.0;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    DcMotorEx highMotor;
    DcMotorEx lowMotor;
    DcMotor intakeMotor;
    DcMotor elevator;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 17;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = -1.0;     // Minimum rotational position
    ElapsedTime myTimer = new ElapsedTime();
    // Define class members
    Servo servo1;
    Servo servo2;
    double position1 = MAX_POS; // Start at beginning position
    double position2 = MIN_POS; // Start at beginning position

    Servo doorLeft;
    Servo doorRight;

    int lastInput = 0;
    int lastInput2 = 0;
    int lastInput3 = 0;
    int lastInput4 = 0;
    int lastInput5 = 0;
    int lastInput6 = 0;

    IMU imu;


    //Variables pour camera
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    ElapsedTime cameraTimer = new ElapsedTime();

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //gamepad1
            drive();
            intake();
            lift();

            //gamepad2
            sorting();
            shooter();

            //automatic
            camera();
        }
    }

    void initialization() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        highMotor = (DcMotorEx) hardwareMap.dcMotor.get("highMotor");
        lowMotor = (DcMotorEx) hardwareMap.dcMotor.get("lowMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        elevator = hardwareMap.dcMotor.get("elevatorMotor");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        doorLeft = hardwareMap.get(Servo.class, "doorLeft");
        doorRight = hardwareMap.get(Servo.class, "doorRight");


        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lowMotor.setDirection(DcMotorEx.Direction.REVERSE);
        highMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        doorLeft.setPosition(1);
        doorRight.setPosition(0);

        initAprilTag();
    }
    void drive() {
        telemetry.addLine("Press A to reset Yaw");
        //telemetry.addLine("Hold left bumper to drive in robot relative");
        //telemetry.addLine("The left joystick sets the robot direction");
        //telemetry.addLine("Moving the right joystick left and right turns the robot");

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
            double frontLeftPower = newForward + newRight + gamepad1.right_stick_x;
            double frontRightPower = newForward - newRight - gamepad1.right_stick_x;
            double backRightPower = newForward + newRight - gamepad1.right_stick_x;
            double backLeftPower = newForward - newRight + gamepad1.right_stick_x;

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

    void intake() {
        if (gamepad1.right_trigger > 0.75) {
            intakeMotor.setPower(0.85);
        } else if (gamepad1.left_trigger > 0.75) {
            intakeMotor.setPower(-0.85);
        } else if (gamepad1.x) {
            intakeMotor.setPower(0);
        }
    }

    void lift() {
        if (gamepad1.right_bumper) {
            elevator.setPower(0.75);
        } else if (gamepad1.left_bumper) {
            elevator.setPower(0);
        }
    }

    void sorting() {
        if (gamepad2.left_trigger > 0.75) {
            doorLeft.setPosition(1);
            doorRight.setPosition(0.33);
        } else if (gamepad2.right_trigger > 0.75) {
            doorLeft.setPosition(0.67);
            doorRight.setPosition(0);
        }
    }

    void shooter() {
        //Shooting wheels (adjusting speed)
        if (gamepad2.dpad_up) {
            if (lastInput == 0) {
                plusOnePower = plusOnePower + 50;
            }
            lastInput = 1;
        } else {
            lastInput = 0;
        }

        if (gamepad2.dpad_down) {
            if (lastInput2 == 0) {
                plusOnePower = plusOnePower - 50;
            }
            lastInput2 = 1;
        } else {
            lastInput2 = 0;
        }

        if (gamepad2.dpad_right) {
            if (lastInput3 == 0) {
                plusOnePower = plusOnePower + 10;
            }
            lastInput3 = 1;
        } else {
            lastInput3 = 0;
        }

        if (gamepad2.dpad_left) {
            if (lastInput4 == 0) {
                plusOnePower = plusOnePower - 10;
            }
            lastInput4 = 1;
        } else {
            lastInput4 = 0;
        }

        if (gamepad2.left_bumper) {
            highMotor.setVelocity(700 + plusOnePower);
            lowMotor.setVelocity(700 + plusOnePower);
        } else if (gamepad2.right_bumper) {
            highMotor.setVelocity(900 + plusOnePower);
            lowMotor.setVelocity(900 + plusOnePower);
        } else if (gamepad2.b) {
            plusOnePower = 0;
            highMotor.setVelocity(0);
            lowMotor.setVelocity(0);
        }


        //Shooting servos
        if (myTimer.milliseconds() >= CYCLE_MS) {
            //servo1 (right)
            if (gamepad2.right_stick_y > 0.75) {
                position1 += INCREMENT;
                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;
                }
            } else if (gamepad2.right_stick_y < -0.75) {
                position1 -= INCREMENT;
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
            } else if (gamepad2.right_stick_button) {
                position1 = MAX_POS;
            }

            //servo2 (left)
            if (gamepad2.left_stick_y < -0.75) {
                position2 += INCREMENT;
                if (position2 >= MAX_POS) {
                    position2 = MAX_POS;
                }
            } else if (gamepad2.left_stick_y > 0.75) {
                position2 -= INCREMENT;
                if (position2 <= MIN_POS) {
                    position2 = MIN_POS;
                }
            } else if (gamepad2.left_stick_button) {
                position2 = MIN_POS;
            }

            servo1.setPosition(position1);
            servo2.setPosition(position2);
            myTimer.reset();
        }

        //advanced (faster or slower shooting rate)
        if (gamepad2.options ) {
            if (gamepad2.dpad_up) {
                if (lastInput5 == 0) {
                    CYCLE_MS = CYCLE_MS + 2;
                }
                lastInput5 = 1;
            } else {
                lastInput5 = 0;
            }
            if (gamepad2.dpad_down) {
                if (lastInput6 == 0) {
                    CYCLE_MS = CYCLE_MS - 2;
                }
                lastInput6 = 1;
            } else {
                lastInput6 = 0;
            }
        }

        //data
        telemetry.addData("Servo 1 Position", "%5.2f", position1);
        telemetry.addData("Servo 2 Position", "%5.2f", position2);
        telemetry.update();
    }

    void camera() {
        if (myTimer.milliseconds() >= 20) {
            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();
            myTimer.reset();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(640,480));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);


        //Close the live feed (illegal in competition)
        visionPortal.stopStreaming();
    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class


