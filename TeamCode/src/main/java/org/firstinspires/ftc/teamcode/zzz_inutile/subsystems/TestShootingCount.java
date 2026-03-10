package org.firstinspires.ftc.teamcode.zzz_inutile.subsystems;

//😎
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class TestShootingCount extends LinearOpMode {
    double plusOnePower = 0.0;
    DcMotorEx highMotor;
    DcMotorEx lowMotor;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 17;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    ElapsedTime myTimer = new ElapsedTime();
    //Automatic shoot
    ElapsedTime automaticShootingTimer = new ElapsedTime();

    boolean yWasPressed = false;
    int ballsShot = 0;
    static final double CYCLETIME_MS = 750;


    // Define class members
    Servo servo1;
    Servo servo2;
    double position1 = MAX_POS; // Start at beginning position
    double position2 = MIN_POS; // Start at beginning position

    boolean dpadUpWasPressed = false;
    boolean dpadDownWasPressed = false;
    boolean dpadRightWasPressed = false;
    boolean dpadLeftWasPressed = false;
    ElapsedTime telemetryTimer = new ElapsedTime();

    //To copy
    double launchVelocity = 0;
    boolean correctLaunchSpeed = false;
    int ballsLaunched = 0;
    //End

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            shooter();
            shootAutomatic();
            allTelemetry();
            countBallsLaunched();
        }
    }

    void initialization() {
        highMotor = (DcMotorEx) hardwareMap.dcMotor.get("highMotor");
        lowMotor = (DcMotorEx) hardwareMap.dcMotor.get("lowMotor");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        highMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowMotor.setDirection(DcMotorEx.Direction.REVERSE);
        highMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    void shooter() {
        //Shooting wheels (adjusting speed)
        if (gamepad2.dpad_up) {
            if (dpadUpWasPressed == false) {
                plusOnePower = plusOnePower + 50;
            }
            dpadUpWasPressed = true;
        } else {
            dpadUpWasPressed = false;
        }

        if (gamepad2.dpad_down) {
            if (dpadDownWasPressed == false) {
                plusOnePower = plusOnePower - 50;
            }
            dpadDownWasPressed = true;
        } else {
            dpadDownWasPressed = false;
        }

        if (gamepad2.dpad_right) {
            if (dpadRightWasPressed == false) {
                plusOnePower = plusOnePower + 10;
            }
            dpadRightWasPressed = true;
        } else {
            dpadRightWasPressed = false;
        }

        if (gamepad2.dpad_left) {
            if (dpadLeftWasPressed == false) {
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

            servo1.setPosition(position1);
            servo2.setPosition(position2);
            myTimer.reset();
        }
    }
    void shootAutomatic() {
        if (gamepad2.y) {
            if (yWasPressed == false) {
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
    private void allTelemetry() {
        if (telemetryTimer.milliseconds() >= 50) {
            telemetry.addData("Servo 1 Position", "%5.2f", position1);
            telemetry.addData("Servo 2 Position", "%5.2f", position2);
            telemetry.addData("High Motor Speed", highMotor.getVelocity());
            telemetry.addData("Lower Motor Speed", lowMotor.getVelocity());
            telemetry.addData("Correct Launch Speed", correctLaunchSpeed);
            telemetry.addData("Balls Launched", ballsLaunched);
            telemetry.update();
            telemetryTimer.reset();
        }
    }   // end method telemetryAprilTag()
    private void countBallsLaunched() {
        if (correctLaunchSpeed == false) {
            if (launchVelocity > 0) {
                if (highMotor.getVelocity() == launchVelocity && lowMotor.getVelocity() == launchVelocity) {
                    correctLaunchSpeed = true;
                }
            }
        }
        if (correctLaunchSpeed) {
            if (highMotor.getVelocity() <= (launchVelocity - 120) && lowMotor.getVelocity() <= (launchVelocity - 120)) {
                ballsLaunched += 1;
                correctLaunchSpeed = false;
            }
        }
    }
}
