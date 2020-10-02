package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Latcher", group="C. Tests System")
//@Disabled
public class TestLatcher extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Latcher latcher = null;

    private double servoDegree  = .5;

    @Override
    public void runOpMode() {
        latcher = new Latcher(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            latcher.manualMoveLatchers(gamepad1.left_trigger, gamepad1.right_trigger);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Degree", latcher.getLeftPosition());
            telemetry.addData("Right Position", latcher.getRightPosition());
            telemetry.update();
            //endregion
        }
    }
}
