package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.components.Motor;
import com.goldenratiorobotics.robot.body.intake.Intake;
import com.goldenratiorobotics.robot.body.shooter.Shooter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Test Intake", group="C. Tests System")
//@Disabled
public class TestIntake extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private Motor intake = null;
    private Motor bottomIntake = null;
    private double speed = 0;





    @Override
    public void runOpMode() {
        intake = Motor.getInstance(hardwareMap,"intakeMotor");
        bottomIntake = Motor.getInstance(hardwareMap, "bottomIntakeMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            speed = -gamepad1.left_stick_y;
            intake.setSpeed(speed);
            bottomIntake.setSpeed(-speed);



            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("speed",speed);
            telemetry.update();
            //endregion
        }
    }
}
