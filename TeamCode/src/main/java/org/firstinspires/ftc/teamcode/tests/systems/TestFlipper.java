package org.firstinspires.ftc.teamcode.tests.systems;

import com.goldenratiorobotics.robot.body.shooter.Shooter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test TestFlipper", group="C. Tests System")
//@Disabled
public class TestFlipper extends LinearOpMode {

    private ElapsedTime runtime    = new ElapsedTime();
    private Shooter     shooter;

    private double position =.5;

    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;


    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

           // if (gamepad1.right_bumper) {
           //     position=Range.clip(position+0.1,0,1);
          //  }
           // if (gamepad1.left_bumper ) {
           //     position=Range.clip(position-0.1,0,1);
           // }
        //&& !previousRightBumper
            //&& !previousLeftBumper
            position = (-gamepad1.left_stick_y+1)/2;


            shooter.moveFlipper(position);

            previousRightBumper = gamepad1.right_bumper;
            previousLeftBumper  =gamepad1.left_bumper;

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("position",shooter.getFlipperPosition());
            telemetry.update();
            //endregion
        }
    }
}
