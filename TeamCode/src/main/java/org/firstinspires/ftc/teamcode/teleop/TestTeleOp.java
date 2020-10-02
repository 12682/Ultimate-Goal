package org.firstinspires.ftc.teamcode.teleop;

import com.goldenratiorobotics.robot.body.claimer.Claimer;
import com.goldenratiorobotics.robot.body.components.SensorColor;
import com.goldenratiorobotics.robot.body.components.SensorRangeREV;
import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.latcher.Latcher;
import com.goldenratiorobotics.robot.body.pincher.Pincher;
import com.goldenratiorobotics.robot.brain.controlprocessor.ControlProcessor;
import com.goldenratiorobotics.robot.brain.gyro.Gyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test TeleOp", group="AB. Test TeleOp")
//@Disabled
public class TestTeleOp extends LinearOpMode {

    private ElapsedTime    runtime     = new ElapsedTime();
    private DriveTrain     driveTrain  = null;
    private Pincher        pincher     = null;
    private Latcher        latcher     = null;
    private Claimer        claimer     = null;
    private Gyro           gyro        = null;
    private SensorRangeREV frontRange  = null;
    private SensorRangeREV backRange   = null;
    private SensorRangeREV leftRange   = null;
    private SensorRangeREV rightRange  = null;
    private SensorColor    bottomColor = null;

    private double rotX          = 0;
    private double moveY         = 0;
    private double moveX         = 0;
    private double speedModifier = 0;
    private String speedMode     = "normal";
    private double multRot       = .33;
    private double multMoveY     = .50;
    private double multMoveX     = .70;

    private double pivotSpeed = 0;
    private double multPivot  = .33;

    @Override
    public void runOpMode() {
        driveTrain  = new DriveTrain(hardwareMap);
        pincher     = new Pincher(hardwareMap);
        latcher     = new Latcher(hardwareMap);
        claimer     = new Claimer(hardwareMap);
        gyro        = new Gyro(hardwareMap);
        frontRange  = SensorRangeREV.getInstance(hardwareMap, "frontRange");
        backRange   = SensorRangeREV.getInstance(hardwareMap, "backRange");
        leftRange   = SensorRangeREV.getInstance(hardwareMap, "leftRange");
        rightRange  = SensorRangeREV.getInstance(hardwareMap, "rightRange");
        bottomColor = SensorColor.getInstance(hardwareMap, "bottomColor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            speedModifier = gamepad1.right_trigger - gamepad1.left_trigger;
            if (speedModifier > 0) {
                speedMode = "fast";
            } else if (speedModifier < 0) {
                speedMode = "slow";
            } else {
                speedMode = "normal";
            }

            multRot   = (.33 + (.25 * speedModifier));
            multMoveY = (.50 + (.25 * speedModifier));
            multMoveX = (.70 + (.25 * speedModifier));

            rotX  = ControlProcessor.squareRoot(gamepad1.left_stick_x * multRot);
            moveY = ControlProcessor.cubeRoot( -gamepad1.left_stick_y * multMoveY);
            moveX = ControlProcessor.cubeRoot(gamepad1.right_stick_x * multMoveX);

            driveTrain.moveTrig(moveX, moveY, rotX);

            if (gamepad1.dpad_up) {
                driveTrain.moveForward(.35);
            }
            if (gamepad1.dpad_down) {
                driveTrain.moveBackward(.35);
            }
            if (gamepad1.dpad_left) {
                driveTrain.moveLeft(.35);
            }
            if (gamepad1.dpad_right) {
                driveTrain.moveRight(.35);
            }

            if (gamepad1.y) {
                latcher.unlatch();
            }
            if (gamepad1.a) {
                latcher.latch();
            }

            pivotSpeed = ControlProcessor.power(gamepad2.left_stick_y * multPivot, 2.0 / 3.0);

            pincher.runPivot(pivotSpeed);

            if (gamepad2.left_bumper) {
                pincher.release();
            }
            if (gamepad2.right_bumper) {
                pincher.pinch();
            }

            if (gamepad2.a) {
                if (gamepad2.right_trigger > .66) {
                    claimer.claim();
                } else {
                    claimer.neuter();
                }
            }

            //region telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", speedMode);
            telemetry.addData("RotX, MoveY, MoveX", rotX + ", " + moveY + ", " + moveX);
            telemetry.addData("PivotSpeed", pivotSpeed);
            telemetry.addData("Yaw, Pitch, Roll", gyro.composeAngleYaw() + ", " + gyro.composeAnglePitch() + ", " + gyro.composeAngleRoll());
            telemetry.addData("Front", frontRange.readCM());
            telemetry.addData("Back", backRange.readCM());
            telemetry.addData("Left", leftRange.readCM());
            telemetry.addData("Right", rightRange.readCM());
            telemetry.addData("Bottom HSV", bottomColor.readHSV()[0] + ", " + bottomColor.readHSV()[1] + ", " + bottomColor.readHSV()[2]);
            telemetry.addData("Encoders", driveTrain.getPositionAll());
            telemetry.addData("Limit0, Limit1", pincher.getValuesLimits()[0] + ", " + pincher.getValuesLimits()[1]);
            telemetry.addData("Limit0, Limit1", pincher.getStatesLimits()[0] + ", " + pincher.getStatesLimits()[1]);
            telemetry.update();
            //endregion
        }
    }
}
