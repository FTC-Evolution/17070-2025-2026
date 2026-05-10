package org.firstinspires.ftc.teamcode.zzz_inutile;

//😎
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometry.DriveToPoint;
import org.firstinspires.ftc.teamcode.odometry.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.Locale;

@Disabled
@Autonomous
public class autonomeForward extends LinearOpMode {
    //Defining Motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx highMotor;
    DcMotorEx lowMotor;
    DcMotor intakeMotor;
    DcMotor elevator;

    //Variables for shooter
    double plusOnePower  = 0.0;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 17;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    ElapsedTime myTimer = new ElapsedTime();

    //Automatic shoot
    ElapsedTime automaticShootingTimer = new ElapsedTime();
    boolean yWasPressed = false;
    int ballsShot = 0;
    static final double CYCLETIME_MS = 2000;


    //Defining Servos
    CRServo servo1;
    CRServo servo2;
    double position1 = MAX_POS; // Start at beginning position
    double position2 = MIN_POS; // Start at beginning position

    //Doors
    Servo doorLeft;
    Servo doorRight;
    int doorSide = 0;

    //Controller Inputs
    boolean dpadUpWasPressed = false;
    boolean dpadDownWasPressed = false;
    boolean dpadRightWasPressed = false;
    boolean dpadLeftWasPressed = false;
    boolean dpadUpWasPressed2 = false;
    boolean dpadDownWasPressed2 = false;
    boolean triggerRightWasPressed = false;
    boolean triggerLeftWasPressed = false;

    //Intake
    int intakePower = 0;

    //IMU
    IMU imu;

    //Variables for camera
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    ElapsedTime cameraTimer = new ElapsedTime();
    int pattern = 0;

    //Telemetry
    ElapsedTime telemetryTimer = new ElapsedTime();

    //Limit Switches
    DigitalChannel limitSwitchLeft;
    DigitalChannel limitSwitchRight;
    boolean limitSwitchLeftWasPressed = false;
    boolean limitSwitchRightWasPressed = false;
    int ballsPassed = 0;

    //Counting shots
    double launchVelocity = 0;
    boolean correctLaunchSpeed = false;
    int ballsReallyLaunched = 0;

    //Odometry
    //Depart dans la loading zone rouge, face au human player
    double xStartingPosition = 63.25;
    double yStartingPosition = -24;
    double headingStartingPosition = 180.00;

    double odoOffsetX = 194.68;
    double odoOffsetY = -20.85;

    double absoluteHeadingToBlueGoal = 0;
    double relativeHeadingToBlueGoal = 0;

    GoBildaPinpointDriver odo;
    DriveToPoint nav;
    Pose2D currentTargetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Pose2D targetPoseShootFar = new Pose2D(DistanceUnit.INCH, 59, -13, AngleUnit.DEGREES, -155.75);
    Pose2D targetPoseShootClose = new Pose2D(DistanceUnit.INCH, -20, 0, AngleUnit.DEGREES, -125.84);
    Pose2D targetPoseEndgame = new Pose2D(DistanceUnit.INCH, 39, 33, AngleUnit.DEGREES, 90);

    ElapsedTime bigTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        waitForStart();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, xStartingPosition, yStartingPosition, AngleUnit.DEGREES,headingStartingPosition));

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                frontLeftDrive.setPower(0.5);
                frontRightDrive.setPower(0.5);
                backLeftDrive.setPower(0.5);
                backRightDrive.setPower(0.5);
            }
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

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        doorLeft = hardwareMap.get(Servo.class, "doorLeft");
        doorRight = hardwareMap.get(Servo.class, "doorRight");

        limitSwitchLeft = hardwareMap.get(DigitalChannel.class, "limitSwitchLeft");
        limitSwitchLeft.setMode(DigitalChannel.Mode.INPUT);
        limitSwitchRight = hardwareMap.get(DigitalChannel.class, "limitSwitchRight");
        limitSwitchRight.setMode(DigitalChannel.Mode.INPUT);

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


        doorLeft.setPosition(0.97);
        doorRight.setPosition(0);

        //Odometry
        nav = new DriveToPoint(this);
        odo = this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(odoOffsetX, odoOffsetY);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);  // <-------À changer
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, xStartingPosition, yStartingPosition, AngleUnit.DEGREES,headingStartingPosition));

        initAprilTag();
    }
    void drive() {
        //telemetry.addLine("Press A to reset Yaw");
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
            if (!triggerRightWasPressed) {
                intakePower += 1;
            }
            triggerRightWasPressed = true;
        } else {
            triggerRightWasPressed = false;
        }

        if (gamepad1.left_trigger > 0.75) {
            if (!triggerLeftWasPressed) {
                intakePower -= 1;
            }
            triggerLeftWasPressed = true;
        } else {
            triggerLeftWasPressed = false;
        }
        if (gamepad1.x) {
            intakePower = 0;
        }

        if (intakePower > 3) {
            intakePower = 3;
        } else if (intakePower < -3) {
            intakePower = -3;
        }

        if (intakePower == 0) {
            intakeMotor.setPower(0);
        } else if (intakePower == 1) {
            intakeMotor.setPower(0.7);
        } else if (intakePower == 2) {
            intakeMotor.setPower(0.85);
        } else if (intakePower == 3) {
            intakeMotor.setPower(1);
        } else if (intakePower == -1) {
            intakeMotor.setPower(-0.7);
        } else if (intakePower == -2) {
            intakeMotor.setPower(-0.85);
        } else if (intakePower == -3) {
            intakeMotor.setPower(-1);
        }
    }

    void lift() {
        if (gamepad1.right_bumper) {
            elevator.setPower(0.75);
        } else if (gamepad1.left_bumper) {
            elevator.setPower(0);
        }
    }

    void odometry() {
        if (gamepad1.dpad_left) {
            currentTargetPose = targetPoseShootFar;
            driveToTarget(currentTargetPose, 0.5);
        }
        if (gamepad1.dpad_right) {
            currentTargetPose = targetPoseShootClose;
            driveToTarget(currentTargetPose, 0.5);
        }
        if (gamepad1.dpad_down) {
            currentTargetPose = targetPoseEndgame;
            driveToTarget(currentTargetPose, 0.5);
        }

        if (gamepad1.ps) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, xStartingPosition, -yStartingPosition, AngleUnit.DEGREES, -headingStartingPosition));
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
        frontLeftDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
        frontRightDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
        backLeftDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
        backRightDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

        String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));
        this.telemetry.addData("TARGET Position", data2);
    }

    void sorting() {
        if (gamepad2.left_trigger > 0.75) {
            doorLeft.setPosition(0.97);
            doorRight.setPosition(0.25);
            doorSide = 2;
        } else if (gamepad2.right_trigger > 0.75) {
            doorLeft.setPosition(0.67);
            doorRight.setPosition(0);
            doorSide = 1;
        }
    }

    void shooter() {
        //Shooting wheels (adjusting speed)
        if (gamepad2.dpad_up) {
            if (!dpadUpWasPressed) {
                plusOnePower = plusOnePower + 50;
            }
            dpadUpWasPressed = true;
        } else {
            dpadUpWasPressed = false;
        }

        if (gamepad2.dpad_down) {
            if (!dpadDownWasPressed) {
                plusOnePower = plusOnePower - 50;
            }
            dpadDownWasPressed = true;
        } else {
            dpadDownWasPressed = false;
        }

        if (gamepad2.dpad_right) {
            if (!dpadRightWasPressed) {
                plusOnePower = plusOnePower + 10;
            }
            dpadRightWasPressed = true;
        } else {
            dpadRightWasPressed = false;
        }

        if (gamepad2.dpad_left) {
            if (!dpadLeftWasPressed) {
                plusOnePower = plusOnePower - 10;
            }
            dpadLeftWasPressed = true;
        } else {
            dpadLeftWasPressed = false;
        }

        if (gamepad2.left_bumper) {
            launchVelocity = 700 + plusOnePower;
        } else if (gamepad2.right_bumper) {
            launchVelocity = 900 + plusOnePower;
        } else if (gamepad2.b) {
            plusOnePower = 0;
            launchVelocity = 0;
        }
        highMotor.setVelocity(launchVelocity);
        lowMotor.setVelocity(launchVelocity);
        servosShooting();
    }

    void servosShooting() {
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

            //servo1.setPosition(position1);
            //servo2.setPosition(position2);
            myTimer.reset();
        }

        //advanced (faster or slower shooting rate)
        if (gamepad2.options ) {
            if (gamepad2.dpad_up) {
                if (!dpadUpWasPressed2) {
                    CYCLE_MS = CYCLE_MS + 2;
                }
                dpadUpWasPressed2 = true;
            } else {
                dpadUpWasPressed2 = false;
            }
            if (gamepad2.dpad_down) {
                if (!dpadDownWasPressed2) {
                    CYCLE_MS = CYCLE_MS - 2;
                }
                dpadDownWasPressed2 = true;
            } else {
                dpadDownWasPressed2 = false;
            }
        }
    }

    void shootAutomatic() {
        if (gamepad2.y) {
            if (!yWasPressed) {
                automaticShootingTimer.reset();
            }
            yWasPressed = true;
            telemetry.addLine("Y is pressed");
            telemetry.update();
            shootIndividual();
        } else {
            yWasPressed = false;
        }
    }

    void shootIndividual() {
        if (automaticShootingTimer.milliseconds() < CYCLETIME_MS) { //servo1 (right)
            telemetry.addLine("0 seconds");
            telemetry.update();
            if (ballsShot == 0) {
                position1 -= 0.4;
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
                //servo1.setPosition(position1);
                ballsShot += 1;
                telemetry.addLine("1 ball shot");
                telemetry.update();
            }
        } else if (automaticShootingTimer.milliseconds() < (2 * CYCLETIME_MS)) { //servo2 (left)
            telemetry.addLine("2 seconds");
            telemetry.update();
            if (ballsShot == 1) {
                position2 = MAX_POS;
                //servo2.setPosition(position2);
                ballsShot  += 1;
                telemetry.addLine("2 balls shot");
                telemetry.update();
            }
        } else { //servo1 (right)
            telemetry.addLine("4 seconds");
            telemetry.update();
            if (ballsShot == 2) {
                position1 = MIN_POS;
                //servo1.setPosition(position1);
                ballsShot += 1;
                telemetry.addLine("3 balls shot");
                telemetry.update();
            }
        }
    }

    void camera() {
        if (cameraTimer.milliseconds() >= 50) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == 21) {
                        pattern = 1;
                    } else if (detection.id == 22) {
                        pattern = 2;
                    } else if (detection.id == 23) {
                        pattern = 3;
                    }
                }
            }
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

    }   // end method initAprilTag()

    void limitSwitches() {
        if (limitSwitchLeft.getState()) {
            if (limitSwitchLeftWasPressed) {
                ballsPassed += 1;
                limitSwitchLeftWasPressed = false;
                doorLeft.setPosition(0.67);
                doorRight.setPosition(0);
                doorSide = 1;
            }
        } else {
            limitSwitchLeftWasPressed = true;
        }

        if (limitSwitchRight.getState()) {
            if (limitSwitchRightWasPressed) {
                ballsPassed += 1;
                limitSwitchRightWasPressed = false;
                doorLeft.setPosition(0.97);
                doorRight.setPosition(0.25);
                doorSide = 2;
            }
        } else {
            limitSwitchRightWasPressed = true;
        }
    }

    private void countBallsLaunched() {
        if (!correctLaunchSpeed) {
            if (launchVelocity > 0) {
                if (highMotor.getVelocity() == launchVelocity && lowMotor.getVelocity() == launchVelocity) {
                    correctLaunchSpeed = true;
                }
            }
        }
        if (correctLaunchSpeed) {
            if (highMotor.getVelocity() <= (launchVelocity - 120) && lowMotor.getVelocity() <= (launchVelocity - 120)) {
                ballsReallyLaunched += 1;
                correctLaunchSpeed = false;
            }
        }
    }

    private void sendingAllTelemetry() {
        if (telemetryTimer.milliseconds() >= 50) {
            telemetry.addData("Servo 1 Position", "%5.2f", position1);
            telemetry.addData("Servo 2 Position", "%5.2f", position2);

            telemetry.addData("High Motor Speed", highMotor.getVelocity());
            telemetry.addData("Lower Motor Speed", lowMotor.getVelocity());

            telemetry.addData("Correct Launch Speed", correctLaunchSpeed);
            telemetry.addData("Balls Launched", ballsReallyLaunched);

            if (limitSwitchLeft.getState()) {
                telemetry.addData("Switch Left", "Open / Not Pressed");
            } else {
                telemetry.addData("Switch Left", "PRESSED / TRIGGERED");
            }

            if (limitSwitchRight.getState()) {
                telemetry.addData("Switch Right", "Open / Not Pressed");
            } else {
                telemetry.addData("Switch Right", "PRESSED / TRIGGERED");
            }
            telemetry.addData("Balls Passed", ballsPassed);

            odo.update();
            telemetry.addData("X Position", "%5.2f", odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", "%5.2f", odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%5.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));

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
            telemetry.addData("Pattern", pattern);
            telemetry.update();
            telemetryTimer.reset();
        }
    }
}   // end class