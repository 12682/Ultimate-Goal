package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.components.ServoNonContinuous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Horizontal Slide Servo V2", group="D. Tests Components")
//@Disabled
public class TestHorizontalSlideServoV2 extends LinearOpMode {

    private ElapsedTime        runtime = new ElapsedTime();
    private ServoNonContinuous servo   = null;

    private double servoPosition = 0;

    private double minPWM = 800;
    private double maxPWM = 2200;

    private double servoMin = (minPWM - 500) / 2000;
    private double servoMax = (maxPWM - 500) / 2000;

    private double servoTurn = (servoMax - servoMin) / 7;

    @Override
    public void runOpMode() {
        servo = ServoNonContinuous.getInstance(hardwareMap, "horizontalServo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            servoPosition = servo.getPosition();

            if (gamepad1.dpad_up) {
                servo.setPosition(Range.clip(servoPosition + (2 * servoTurn), servoMin, servoMax));
            }
            if (gamepad1.dpad_down) {
                servo.setPosition(Range.clip(servoPosition - (2 * servoMin), servoMin, servoMax));
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo position", servo.getPosition());
            telemetry.addData("Servo Range", servoMin + "-" + servoMax);
            telemetry.update();
            //endregion
        }
    }
}
