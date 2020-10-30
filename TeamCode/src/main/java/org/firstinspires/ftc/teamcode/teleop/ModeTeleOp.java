package org.firstinspires.ftc.teamcode.teleop;


import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;


import com.goldenratiorobotics.robot.body.intake.Intake;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Mode", group="A. ModeTeleOp")
//@Disabled
public class ModeTeleOp extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Intake         intake      = null;

    private double rotX          = 0;
    private double moveY         = 0;
    private double moveX         = 0;
    private double speedModifier = 0;



    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        intake      = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                speedModifier = .4;
            } else {
                speedModifier = gamepad1.right_trigger - gamepad1.left_trigger;
            }

            moveX  = ControlProcessor.squareRoot(gamepad1.left_stick_x * speedModifier);
            moveY = ControlProcessor.cubeRoot( -gamepad1.left_stick_y * speedModifier);
            rotX = ControlProcessor.cubeRoot(gamepad1.right_stick_x * speedModifier);

            driveTrain.moveTrig(moveX, moveY, rotX);

            if (gamepad1.dpad_up) {
                driveTrain.moveForward(.40);
            }
            if (gamepad1.dpad_down) {
                driveTrain.moveBackward(.40);
            }
            if (gamepad1.dpad_left) {
                driveTrain.moveLeft(.40);
            }
            if (gamepad1.dpad_right) {
                driveTrain.moveRight(.40);
            }

            if (gamepad2.b) {
                intake.takeOut();

            } else if (gamepad2.x) {
                intake.takeIn();

            } else{
                intake.stop();
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", speedModifier);
            telemetry.update();
            //endregion
        }
    }
}
