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


@TeleOp
public class NewGrosBot extends LinearOpMode {
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
    static ElapsedTime myTimer = new ElapsedTime();
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
        if (gamepad2.options) {
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
}

