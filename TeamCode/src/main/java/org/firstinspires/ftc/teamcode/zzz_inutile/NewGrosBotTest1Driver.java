package org.firstinspires.ftc.teamcode.zzz_inutile;

//😎
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp
public class NewGrosBotTest1Driver extends LinearOpMode {
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
    static final double MIN_POS = 0.0;     // Minimum rotational position
    ElapsedTime myTimer = new ElapsedTime();

    //Automatic Shoot
    ElapsedTime automaticShootingTimer = new ElapsedTime();
    boolean xWasPressed = false;
    int ballsShot = 0;
    static final double CYCLETIME_MS = 750;


    // Define class members
    Servo servo1;
    Servo servo2;
    double position1 = MAX_POS; // Start at beginning position
    double position2 = MIN_POS; // Start at beginning position

    Servo doorLeft;
    Servo doorRight;

    boolean dpadUpWasPressed = false;
    boolean dpadDownWasPressed = false;
    boolean dpadRightWasPressed = false;
    boolean dpadLeftWasPressed = false;
    boolean dpadUpWasPressed2 = false;
    boolean dpadDownWasPressed2 = false;
    boolean yWasPressed = false;
    int playerNumber = 1;

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

    ElapsedTime telemetryTimer = new ElapsedTime();

    DigitalChannel limitSwitchLeft;
    DigitalChannel limitSwitchRight;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            playerChanging();

            if (playerNumber == 1) {
                drive();
                intake();
                shooter();
            }

            if (playerNumber == 2) {
                sorting();
                servosShooting();
                shootAutomatic();
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

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
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


        doorLeft.setPosition(0.67);
        doorRight.setPosition(0);
    }

    void playerChanging() {
        if (gamepad1.y) {
            if (yWasPressed == false) {
                if (playerNumber == 1) {
                    playerNumber = 2;
                } else if (playerNumber == 2) {
                    playerNumber = 1;
                    /*position1 = MAX_POS;
                    position2 = MIN_POS;
                    servo1.setPosition(position1);
                    servo2.setPosition(position2); */

                }
                telemetry.addData("Player ", playerNumber);
                telemetry.update();
            }
            yWasPressed = true;
        } else {
            yWasPressed = false;
        }
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
            intakeMotor.setPower(0.85);
        } else if (gamepad1.left_trigger > 0.75) {
            intakeMotor.setPower(-0.85);
        } else if (gamepad1.x) {
            intakeMotor.setPower(0);
        }
    }

    void sorting() {
        if (gamepad1.left_trigger > 0.75) {
            doorLeft.setPosition(1);
            doorRight.setPosition(0.33);
        } else if (gamepad1.right_trigger > 0.75) {
            doorLeft.setPosition(0.67);
            doorRight.setPosition(0);
        }
    }

    void shooter() {
        //Shooting wheels (adjusting speed)
        if (gamepad1.dpad_up) {
            if (dpadUpWasPressed == false) {
                plusOnePower = plusOnePower + 50;
            }
            dpadUpWasPressed = true;
        } else {
            dpadUpWasPressed = false;
        }

        if (gamepad1.dpad_down) {
            if (dpadDownWasPressed == false) {
                plusOnePower = plusOnePower - 50;
            }
            dpadDownWasPressed = true;
        } else {
            dpadDownWasPressed = false;
        }

        if (gamepad1.dpad_right) {
            if (dpadRightWasPressed == false) {
                plusOnePower = plusOnePower + 10;
            }
            dpadRightWasPressed = true;
        } else {
            dpadRightWasPressed = false;
        }

        if (gamepad1.dpad_left) {
            if (dpadLeftWasPressed == false) {
                plusOnePower = plusOnePower - 10;
            }
            dpadLeftWasPressed = true;
        } else {
            dpadLeftWasPressed = false;
        }

        if (gamepad1.left_bumper) {
            highMotor.setVelocity(700 + plusOnePower);
            lowMotor.setVelocity(700 + plusOnePower);
        } else if (gamepad1.right_bumper) {
            highMotor.setVelocity(900 + plusOnePower);
            lowMotor.setVelocity(900 + plusOnePower);
        } else if (gamepad1.b) {
            plusOnePower = 0;
            highMotor.setVelocity(0);
            lowMotor.setVelocity(0);
        }
    }

    void servosShooting() {
        if (myTimer.milliseconds() >= CYCLE_MS) {
            //servo1 (right)
            if (gamepad1.right_stick_y > 0.75) {
                position1 += INCREMENT;
                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;
                }
            } else if (gamepad1.right_stick_y < -0.75) {
                position1 -= INCREMENT;
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
            } else if (gamepad1.right_stick_button) {
                position1 = MAX_POS;
            }

            //servo2 (left)
            if (gamepad1.left_stick_y < -0.75) {
                position2 += INCREMENT;
                if (position2 >= MAX_POS) {
                    position2 = MAX_POS;
                }
            } else if (gamepad1.left_stick_y > 0.75) {
                position2 -= INCREMENT;
                if (position2 <= MIN_POS) {
                    position2 = MIN_POS;
                }
            } else if (gamepad1.left_stick_button) {
                position2 = MIN_POS;
            }

            servo1.setPosition(position1);
            servo2.setPosition(position2);
            myTimer.reset();
        }

        //advanced (faster or slower shooting rate)
        if (gamepad1.options ) {
            if (gamepad1.dpad_up) {
                if (dpadUpWasPressed2 == false) {
                    CYCLE_MS = CYCLE_MS + 2;
                }
                dpadUpWasPressed2 = true;
            } else {
                dpadUpWasPressed2 = false;
            }
            if (gamepad1.dpad_down) {
                if (dpadDownWasPressed2 == false) {
                    CYCLE_MS = CYCLE_MS - 2;
                }
                dpadDownWasPressed2 = true;
            } else {
                dpadDownWasPressed2 = false;
            }
        }
    }
    void shootAutomatic() {
        if (gamepad1.x) {
            if (xWasPressed == false) {
                automaticShootingTimer.reset();
            }
            xWasPressed = true;
            telemetry.addLine("Y is pressed");
            telemetry.update();
            shootIndividual();
        } else {
            xWasPressed = false;
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
                servo1.setPosition(position1);
                ballsShot += 1;
                telemetry.addLine("1 ball shot");
                telemetry.update();
            }
        } else if (automaticShootingTimer.milliseconds() < (2 * CYCLETIME_MS)) { //servo2 (left)
            telemetry.addLine("2 seconds");
            telemetry.update();
            if (ballsShot == 1) {
                position2 = MAX_POS;
                servo2.setPosition(position2);
                ballsShot  += 1;
                telemetry.addLine("2 balls shot");
                telemetry.update();
            }
        } else { //servo1 (right)
            telemetry.addLine("4 seconds");
            telemetry.update();
            if (ballsShot == 2) {
                position1 = MIN_POS;
                servo1.setPosition(position1);
                ballsShot += 1;
                telemetry.addLine("3 balls shot");
                telemetry.update();
            }
        }
    }
}   // end class


