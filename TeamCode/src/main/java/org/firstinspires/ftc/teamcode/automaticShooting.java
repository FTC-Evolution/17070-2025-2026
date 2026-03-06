package org.firstinspires.ftc.teamcode;

//😎
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

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;


@TeleOp
public class NewGrosBot extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    int CYCLE_MS = 17;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    ElapsedTime myTimer = new ElapsedTime();
    // Define class members
    Servo servo1;
    Servo servo2;
    double position1 = MAX_POS; // Start at beginning position
    double position2 = MIN_POS; // Start at beginning position

    ElapsedTime automaticShootingTimer = new ElapsedTime(); 

    boolean yWasPressed = false;   

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            servosShooting();
            automaticTest();
        }
    }

    void initialization() {
        
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
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

    void automaticTest() {
        if (myTimer.milliseconds() >= CYCLE_MS) {
            if (gamepad2.y) {
                if (yWasPressed == false) {
                    automaticShootingTimer.reset();
                }
                    yWasPressed = true;
                    
                    //servo1 (right)
                    if (automaticShootingTimer.milliseconds < 2000) {
                        position1 += INCREMENT;
                        if (position1 >= MAX_POS) {
                            position1 = MAX_POS;
                        }
                    }

                    //servo2 (left)
                    if (automaticShootingTimer.milliseconds > 2000 && automaticShootingTimer.milliseconds < 4000) {
                        position2 -= INCREMENT;
                        if (position2 <= MIN_POS) {
                            position2 = MIN_POS;
                        }
                    }

                    //servo1 (right)
                    if (automaticShootingTimer.milliseconds > 4000) {
                        position1 += INCREMENT;
                        if (position1 >= MAX_POS) {
                            position1 = MAX_POS;
                        }
                    }

                    servo1.setPosition(position1);
                    servo2.setPosition(position2);
                    myTimer.reset();
            } else {
                yWasPressed = false;
            }
        }
    }
}