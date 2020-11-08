package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.shooter.Shooter;
import com.goldenratiorobotics.robot.body.wobbleGrabber.WobbleGrabber;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test WobbleGrabber", group="C. Tests System")
//@Disabled
public class TestWobbleGrabber extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private WobbleGrabber wobbleGrabber;
    private double position =.5;

    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;


    @Override
    public void runOpMode() {
        wobbleGrabber = new WobbleGrabber(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

//            if (gamepad1.right_bumper && !previousRightBumper) {
//                position=Range.clip(position+0.1,0,1);
//            }
//            if (gamepad1.left_bumper && !previousLeftBumper) {
//                position=Range.clip(position-0.1,0,1);
//            }
            position=gamepad1.left_trigger;
            wobbleGrabber.pinchToPosition(position);

            previousRightBumper = gamepad1.right_bumper;
            previousLeftBumper  =gamepad1.left_bumper;

            wobbleGrabber.runArm(-gamepad1.left_stick_y);

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("position",wobbleGrabber.getPincherPosition());
            telemetry.addData("limitIn",wobbleGrabber.isArmIn());
            telemetry.addData("limitOut",wobbleGrabber.isArmOut());
            telemetry.update();
            //endregion
        }
    }
}
