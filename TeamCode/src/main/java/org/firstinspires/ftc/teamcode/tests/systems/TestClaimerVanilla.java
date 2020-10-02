package org.firstinspires.ftc.teamcode.tests.systems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Claimer Basic", group="C. Tests System")
//@Disabled
public class TestClaimerVanilla extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo       claimer = null;


    @Override
    public void runOpMode() {
        claimer = hardwareMap.get(Servo.class, "claimer");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                claimer.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                claimer.setPosition(0);
            }


            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Position", claimer.getPosition());
            telemetry.update();
            //endregion
        }
    }
}
