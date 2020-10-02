package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Pincher Servo", group="D. Tests Components")
//@Disabled
public class TestPincherServo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Pincher     pincher = null;

    private int    step          = 0;
    private double servoPosition = .5;

    @Override
    public void runOpMode() {
        pincher = new Pincher(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (step == 0) {
                pincher.moveServoManual(servoPosition);
                servoPosition = 0;
                step = 1;
                sleep(1000);
            }
            if (step == 1) {
                pincher.moveServoManual(servoPosition);
                servoPosition = .5;
                step = 2;
                sleep(1000);
            }
            if (step == 2) {
                pincher.moveServoManual(servoPosition);
                servoPosition = 1;
                step = 3;
                sleep(1000);
            }
            if (step == 3) {
                pincher.moveServoManual(servoPosition);
                servoPosition = .5;
                step = 0;
                sleep(1000);
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Step", step);
            telemetry.addData("Servo position", servoPosition);
            telemetry.update();
            //endregion
        }
    }
}
