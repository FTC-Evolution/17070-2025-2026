package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class PIDFTuning extends OpMode {
    public DcMotorEx fLywheelMotor;

    public double highVelocity = 1500;

    public double LowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0; 

    double stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        fLywheelMotor = hardwareMap.get(DcMotorEx.class, "motor");
        fLywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        fLywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry addLine("Init complete");
    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVeLocity) {
            curTargetVelocity = LowVeLocity;
            } else { curTargetVelocity = highVelocity; }
        }
        
        if (gamepad1. bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        // set new PIDE coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        fLywheeLMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        fLywheelMotor.setVelocity(curTargetVeLocity);

        double curVelocity = flywheelMotor.getVelocity;
        double error = curTargetVelocity - curVelocity;


        teLemetry.addData("Target Velocity", curTargetVeLocity);
        telemetry.addData("Current Velocity", "%. 2f", curVelocity);
        teLemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------")
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
