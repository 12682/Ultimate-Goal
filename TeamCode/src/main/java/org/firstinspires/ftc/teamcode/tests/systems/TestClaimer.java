package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.claimer.Claimer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test Claimer", group="C. Tests System")
//@Disabled
public class TestClaimer extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Claimer     claimer = null;

    private double servoDegree  = .5;

    @Override
    public void runOpMode() {
        claimer = new Claimer(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servoDegree = Range.clip(servoDegree + .1, 0, 1);
            }
            if (gamepad1.dpad_down) {
                servoDegree = Range.clip(servoDegree - .1, 0, 1);
            }

            claimer.moveServoManual(servoDegree);

            if (gamepad1.y) {
                claimer.setClaimPosition(servoDegree);
            }
            if (gamepad1.a) {
                claimer.setNeutralPosition(servoDegree);
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Degree", claimer.getServoPosition());
            telemetry.addData("Claim Position", claimer.getClaimPosition());
            telemetry.addData("Neutral Position", claimer.getNeutralPosition());
            telemetry.update();
            //endregion
        }
    }
}
