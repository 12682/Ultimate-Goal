package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Intake V2", group="C. Tests System")
//@Disabled
public class TestIntakeV2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Motor       intakeMotor1;
    private Motor       intakeMotor2;

    private double maxSpeed = .45;
    private double speed    = 0;

    @Override
    public void runOpMode() {
        intakeMotor1 = Motor.getInstance(hardwareMap, "intakeMotor1");
        intakeMotor2 = Motor.getInstance(hardwareMap, "intakeMotor2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            speed = maxSpeed * (gamepad1.right_trigger - gamepad1.left_trigger);

            intakeMotor1.setSpeed(speed);
            intakeMotor2.setSpeed(-speed);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed / Max Speed", speed + " / " + maxSpeed);
            telemetry.addData("Encoder 1", intakeMotor1.getPosition());
            telemetry.addData("Encoder 2", intakeMotor2.getPosition());
            telemetry.update();
            //endregion
        }
    }
}
