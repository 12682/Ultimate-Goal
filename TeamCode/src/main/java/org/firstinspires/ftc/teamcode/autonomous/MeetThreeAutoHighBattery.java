package org.firstinspires.ftc.teamcode.autonomous;

import com.goldenratiorobotics.robot.body.drivetrain.DriveTrain;
import com.goldenratiorobotics.robot.body.odometry.OdometryUnit;
import com.goldenratiorobotics.robot.body.shooter.Shooter;
import com.goldenratiorobotics.robot.body.wobbleGrabber.WobbleGrabber;
import com.goldenratiorobotics.robot.brain.smart.SmartOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Meet Three Auto High Battery", group = "A. Autonomous")
//@Disabled
public class MeetThreeAutoHighBattery extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private SmartOdometry smartOdometry;
    private Shooter        shooter     = null;
    private double shooterSpeed        =.70;
    private WobbleGrabber wobbleGrabber = null;
    int ringNumber = 0;
    int stage = 0;
    int endLoop = 0;
    long timeout = 2000;
    long flipperTimeOut = 1000;
    long st = 0;
    OpenCvInternalCamera phoneCam;
    MeetThreeAutoHighBattery.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        shooter     = new Shooter(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "rightBack", "leftFront", "rightFront");
        smartOdometry = new SmartOdometry(driveTrain,odometryUnit);
        wobbleGrabber =new WobbleGrabber(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new MeetThreeAutoHighBattery.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        odometryUnit.start();

        wobbleGrabber.isPinched();
        while (opModeIsActive()) {
            //Count rings
            if (stage==0){
                pipeline.getAnalysis();
                wobbleGrabber.pinch();
                ringNumber = pipeline.getRingNumber();
                stage++;
            }
            //move forward slightly and right to get out of the way of rings and to be in front of shooting location.
            if (stage==1){
                    smartOdometry.moveForward(DistanceUnit.CM,40,.5,.8,50);
                while (System.currentTimeMillis() -st < 500){
                }
                    smartOdometry.moveRight(DistanceUnit.CM,70,.5,.8,3000);
                stage++;
            }

            //Move forward to be parallel to shooting position.

            if (stage==2){
                    smartOdometry.moveForward(DistanceUnit.CM,170, .5, .8, 1300);
                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 1000){
                }
                stage++;
            }
          //  Move left into shooting location
            if (stage==3){
                    smartOdometry.moveLeft(DistanceUnit.CM, 33, .2, .5, 3000);

                stage++;
            }
            //Shooting
            if (stage==4) {
                //start motor for shooting.
                shooter.runShooter(shooterSpeed);
                // wait loop for motor to come to speed. wait 3 sec
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 3000) {
                }
                st = System.currentTimeMillis();
                // Flip to shoot ring 1
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.flipIn();
                }
                st = System.currentTimeMillis();
                // return flipper to neuter position
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.neuterFlipper();
                }
                st = System.currentTimeMillis();
                // Flip to shoot ring 2
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.flipIn();
                }
                st = System.currentTimeMillis();
                // return flipper to neuter position
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.neuterFlipper();
                }
                st = System.currentTimeMillis();
                // Flip to shoot ring 3
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.flipIn();
                }
                st = System.currentTimeMillis();
                // return flipper to neuter position
                while (System.currentTimeMillis() - st < flipperTimeOut) {
                    shooter.neuterFlipper();
                }
                shooter.runShooter(0);
                stage++;
            }
            //move to delivery zone.
            if (stage==5) {
                if (ringNumber == 0) {
                    smartOdometry.moveForward(DistanceUnit.CM, 70, .2, .5, 2100);
                    smartOdometry.moveLeft(DistanceUnit.CM, 40, .2, .5, 4000);
                }

                if (ringNumber == 4) {
                    smartOdometry.moveForward(DistanceUnit.CM,200,.2,.6,4500);
                    smartOdometry.moveLeft(DistanceUnit.CM, 60, .2, .6, 1800);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveForward(DistanceUnit.CM,65,.2,.5,3700);
                    smartOdometry.moveRight(DistanceUnit.CM, 10, .2, .5,500 );
                }
                stage++ ;
            }

            //drop wobble goal
            if (stage == 6) {
                wobbleGrabber.runArmManual(.6);
                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 1700){

                }
                wobbleGrabber.release();
//                st = System.currentTimeMillis();
//                while (System.currentTimeMillis() - st < timeout) {
//                    wobbleGrabber.runArm(-.4); }
                stage++;
            }
            //close pincher and bring arm in
            if (stage == 7) {
                wobbleGrabber.release();
                wobbleGrabber.runArmManual(-.7);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 2000) {
                }

//                st = System.currentTimeMillis();
//                while (System.currentTimeMillis() - st < timeout) {
//                    wobbleGrabber.runArmManual(.4);
//                 }
//                wobbleGrabber.runArm(0);
//                wobbleGrabber.pinch();
//                wobbleGrabber.release();
                stage++;
            }
            //Shoot


            //Park
            if (stage == 8) {
                if (ringNumber == 4) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 60, .2, .8, 2300);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 35, .2, .8, 100);
                }
                stage++;
            }

            //region telemetry
            telemetry.addData("Ring number =", + ringNumber);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Vertical Left Position", odometryUnit.returnVL());
            telemetry.addData("Vertical Right Position", odometryUnit.returnVR());
            telemetry.addData("Horizontal Position", odometryUnit.returnH());
            telemetry.addData("(X, Y, Theta)", odometryUnit.returnPoint().toString());
            telemetry.addData("(X, Y) IN", odometryUnit.returnPointUnits(DistanceUnit.INCH).toString());
            telemetry.addData("(X, Y) CM", odometryUnit.returnPointUnits(DistanceUnit.CM).toString());
            telemetry.addData("Thread is Active", odometryUnit.isAlive());
            telemetry.update();
            //endregion
        }

        odometryUnit.stop();
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140,15);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition position = MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

        public int getRingNumber(){
            int ringNum;
            if (position == MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.FOUR){
                ringNum = 4;
            } else if (position == MeetThreeAutoHighBattery.SkystoneDeterminationPipeline.RingPosition.ONE){
                ringNum = 1;
            } else {
                ringNum =0;
            }
            return ringNum;
        }
    }
}
