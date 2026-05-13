package org.firstinspires.ftc.teamcode.zzz_inutile;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class Essai extends OpMode {

    @Override
    public void init() {
        String nummer = "17017";

        telemetry.addData("Nb:", nummer);

    }

    @Override
    public void loop() {

        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        boolean buttonA = gamepad1.a;

        telemetry.addData("joystickX:", joyX);
        telemetry.addData("joystickY:", joyY);

        if (buttonA) {
            telemetry.addData("A:", "Ya");
        } else {
            telemetry.addData("A:", "Nein");
        }

    }

}
