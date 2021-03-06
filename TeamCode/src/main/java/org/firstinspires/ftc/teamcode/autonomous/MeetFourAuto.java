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

@Autonomous(name="Meet Four Auto", group = "A. Autonomous")
//@Disabled
public class MeetFourAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain;
    private OdometryUnit odometryUnit;
    private SmartOdometry smartOdometry;
    private Shooter        shooter     = null;
    private double shooterSpeed        =.65;
    private WobbleGrabber wobbleGrabber = null;
    int ringNumber = 0;
    int stage = 0;
    int endLoop = 0;
    long timeout = 2000;
    long flipperTimeOut = 1000;
    long st = 0;
    OpenCvInternalCamera phoneCam;
    MeetFourAuto.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap);
        shooter     = new Shooter(hardwareMap);
        odometryUnit = new OdometryUnit(hardwareMap, "rightBack", "leftFront", "rightFront");
        smartOdometry = new SmartOdometry(driveTrain,odometryUnit);
        wobbleGrabber =new WobbleGrabber(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new MeetFourAuto.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though,
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

       // wobbleGrabber.isPinched();
        while (opModeIsActive()) {
           // wobbleGrabber.pinch();
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
                    smartOdometry.moveRight(DistanceUnit.CM,79,.5,.8,1750);
                stage++;
            }

            //Move forward to be parallel to shooting position.
            if (stage==2){
                smartOdometry.moveForward(DistanceUnit.CM,258, .5, .8, 1400);                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 1000){
                }
                stage++;
            }
          //  Move left into shooting location
            if (stage==3){
                smartOdometry.moveLeft(DistanceUnit.CM, 65, .3, .8, 1600);


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
                    smartOdometry.moveForward(DistanceUnit.CM, 90, .2, .5, 1000);
                    smartOdometry.moveLeft(DistanceUnit.CM, 15, .2, .5, 1000);
                }

                if (ringNumber == 4) {
                    smartOdometry.moveForward(DistanceUnit.CM,200,.3,.6,2900);
                    smartOdometry.moveLeft(DistanceUnit.CM, 25, .3, .6, 700);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveRight(DistanceUnit.CM,42, .3,.6,1000);
                    smartOdometry.moveForward(DistanceUnit.CM,45,.3,.6,1800);
                    //smartOdometry.moveRight(DistanceUnit.CM, 1, .2, .5,500 );
                    telemetry.addData("i",shooter.getSpeed());
                }
                stage++;
            }

            //drop wobble goal and release
            if (stage == 6) {
                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 500){
                    wobbleGrabber.runArmManual(.6);
                    wobbleGrabber.pinch();
                }
               wobbleGrabber.runArmManual(-.6);
                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 500){
                }
               // wobbleGrabber.release();
                st=System.currentTimeMillis();
                while (System.currentTimeMillis() -st < 1500){
                }
               wobbleGrabber.release();
                stage++;
            }
            //close pincher and bring arm in
            if (stage == 7) {
                wobbleGrabber.runArmManual(.6);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 2000) {
                }
                wobbleGrabber.runArm(0);
                wobbleGrabber.pinch();
                stage++;
            }
            //Park
            if (stage == 8) {
                if (ringNumber == 4) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 60, .2, .8, 2200);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 50, .2, .8, 1200);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 20, .2, .8, 200);
                }
                stage=999;
            }
            //strafe right
            if (stage == 9) {
                if (ringNumber == 4) {
                    smartOdometry.moveRight(DistanceUnit.CM, 60, .2, .8, 2300);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveRight(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveRight(DistanceUnit.CM, 35, .2, .8, 1350);
                }
                stage++;
            }
            //move backwards to wobble goal
            if (stage == 10) {
                if (ringNumber == 4) {
                smartOdometry.moveBackward(DistanceUnit.CM, 160, .2, .8, 5000);
            }
                if (ringNumber == 1) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 35, .2, .8, 1400);
                }
                stage++;
            }
            //open the pincher
            if (stage == 11) {
                wobbleGrabber.release();
                stage++;
            }

            //put the arm down
            if (stage == 12) {
                wobbleGrabber.runArmManual(.7);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 2000) {
                }
                wobbleGrabber.runArm(0);
                stage++;
            }

            //grab wobble grab
            if (stage == 13)  {
                wobbleGrabber.pinch();
                stage++;
            }

            //slightly lift the arm
            if (stage == 14) {
                wobbleGrabber.runArmManual(-.7);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 500) {
                }
                wobbleGrabber.runArm(0);
                stage++;
            }

            //move forward twords target position
            if (stage == 15) {
                if (ringNumber == 4) {
                    smartOdometry.moveForward(DistanceUnit.CM, 60, .2, .8, 2300);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveForward(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveForward(DistanceUnit.CM, 35, .2, .8, 350);
                }
                stage++;
            }

            //move left to target position
            if (stage == 16) {
                if (ringNumber == 4) {
                    smartOdometry.moveRight(DistanceUnit.CM, 60, .2, .8, 2300);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveRight(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveRight(DistanceUnit.CM, 35, .2, .8, 350);
                }
                stage++;
            }

            //drops wobble goal and release
            if (stage == 17) {
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 2000) {
                    wobbleGrabber.runArmManual(.6);
                    wobbleGrabber.pinch();
                }
                wobbleGrabber.runArmManual(0);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 500) {
                }
                wobbleGrabber.release();
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 500) {
                }
            }

            //closes grabber and brings arm in
            if (stage == 18) {
                wobbleGrabber.runArmManual(-.7);
                st = System.currentTimeMillis();
                while (System.currentTimeMillis() - st < 2000) {
                }
                wobbleGrabber.runArm(0);
                wobbleGrabber.pinch();
                stage++;
            }

            //park
            if (stage == 18) {
                if (ringNumber == 4) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 60, .2, .8, 2300);
                }
                if (ringNumber == 1) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 50, .2, .8, 1900);
                }
                if (ringNumber == 0) {
                    smartOdometry.moveBackward(DistanceUnit.CM, 35, .2, .8, 350);
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(95,13);

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
        private volatile MeetFourAuto.SkystoneDeterminationPipeline.RingPosition position = MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.FOUR;

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

            position = MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.NONE;
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
            if (position == MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.FOUR){
                ringNum = 4;
            } else if (position == MeetFourAuto.SkystoneDeterminationPipeline.RingPosition.ONE){
                ringNum = 1;
            } else {
                ringNum =0;
            }
            return ringNum;
        }
    }
}
