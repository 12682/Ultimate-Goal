package org.firstinspires.ftc.teamcode.teleop;


import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;


import com.goldenratiorobotics.robot.body.intake.Intake;
import com.goldenratiorobotics.robot.body.shooter.Shooter;
import com.goldenratiorobotics.robot.body.wobbleGrabber.WobbleGrabber;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Mode", group="A. ModeTeleOp")
//@Disabled
public class ModeTeleOp extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Intake         intake      = null;
    private Shooter        shooter     = null;
    private WobbleGrabber  wobbleGrabber = null;

    private double rotX          = 0;
    private double moveY         = 0;
    private double moveX         = 0;
    private double speedModifier = 0;
    private boolean previousGamepad2Y = false;
    private boolean previousGamepad2LB = false;
    private boolean previousGamepad2RB = false;
    private double shooterSpeed        =.75;
    private boolean isShooting         = false;
    private boolean previousGamepad2A   = false;



    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        intake      = new Intake(hardwareMap);
        shooter     = new Shooter(hardwareMap);
        wobbleGrabber =new WobbleGrabber(hardwareMap);
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

            moveX  = ControlProcessor.squareRoot(gamepad1.left_stick_x * speedModifier  );
            moveY = ControlProcessor.cubeRoot(-gamepad1.left_stick_y * speedModifier);
            rotX = ControlProcessor.cubeRoot(gamepad1.right_stick_x * speedModifier);

            driveTrain.moveTrig(moveX, moveY, rotX);

            if (gamepad1.dpad_up) {
                driveTrain.moveBackward(.40);
            }
            if (gamepad1.dpad_down) {
                driveTrain.moveForward(.40);
            }
            if (gamepad1.dpad_left) {
                driveTrain.moveLeft(.40);
            }
            if (gamepad1.dpad_right) {
                driveTrain.moveRight(.40);
            }

            if (gamepad2.b) {
                intake.takeIn();

            } else if (gamepad2.x) {
                intake.takeOut();

            } else{
                intake.stop();
            }
            if (gamepad2.left_stick_y >.2){
                shooter.flipIn();
            } else {
                shooter.neuterFlipper();
            }

            //shooterSpeed = gamepad2.right_trigger;
            //if right trigger is pulled start shooter
            if (gamepad2.right_trigger >.5) {
                shooter.runShooter(Range.clip(shooterSpeed, 0, 1));
            }
            //shooter.runShooter(Range.clip(shooterSpeed, 0, .70));

            //if left trigger is puled, stop shooter
            if (gamepad2.left_trigger >.5) {
                shooter.runShooter(0);
            }

            //if right bumper is pressed, increase speed by .5
            if (gamepad2.right_bumper=true) {
                shooterSpeed = shooterSpeed + .05;
               shooter.runShooter(Range.clip(shooterSpeed, 0, 1));
            }

            //if left bumper is pressed, decrease speed by .5
            if (gamepad2.left_bumper=true) {
                shooterSpeed = shooterSpeed - .05;
                shooter.runShooter(Range.clip(shooterSpeed, 0, 1));
            }

            if (gamepad2.dpad_up){
                wobbleGrabber.runArmManual(- .4);
            } else if (gamepad2.dpad_down) {
                wobbleGrabber.runArmManual(.4);
            } else {
                wobbleGrabber.runArmManual(gamepad2.right_stick_y);
            }


            if (gamepad2.a && !previousGamepad2A) {
                if (wobbleGrabber.isPinched()){
                    wobbleGrabber.release();
                } else {
                    wobbleGrabber.pinch();
                }
            }


            previousGamepad2Y = gamepad2.y;
            previousGamepad2LB = gamepad2.left_bumper;
            previousGamepad2RB = gamepad2.right_bumper;
            previousGamepad2A  =gamepad2.a;


            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("shooter Speed", shooterSpeed);
            telemetry.update();
            //endregion
        }
    }
}