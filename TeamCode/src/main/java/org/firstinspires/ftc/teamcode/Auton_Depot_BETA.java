package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@Autonomous
public class Auton_Depot_BETA extends eBotsOpMode {

    //This autonomous mode performs a sequence of tasks intended to :
    // move rover from the depot latch on the lander
    // sample
    // claim depot
    // park in crater
    // detailed list of operations is listed in opMode main body


    //  DEFINE CONSTANTS FOR THE ROBOT
    //  THESE ARE ALL POSITIONS ASSUMED THE ROBOT IS COMPLETELY FOLDED PRIOR TO START OF AUTON

    final static double SAMPLING_DRIVE_COMPONENT = 0.85;
    final static double DEPOT_DRIVE_COMPONENT = .4;
    final static String VUFORIA_KEY = "AdGgXjv/////AAABmSSQR7vFmE3cjN2PqTebidhZFI8eL1qz4JblkX3JPyyYFRNp/Su1RHcHvkTzJ1YjafcDYsT0l6b/2U/fEZObIq8Si3JYDie2PfMRfdbx1+U0supMRZFrkcdize8JSaxMeOdtholJ+hUZN+C4Ovo7Eiy/1sBrqihv+NGt1bd2/fXwvlIDJFm5lJHF6FCj9f4I7FtIAB0MuhdTSu4QwYB84m3Vkx9iibTUB3L2nLLtRYcbVpoiqvlxvZomUd2JMef+Ux6+3FA3cPKCicVfP2psbjZrxywoc8iYUAq0jtsEaxgFdYoaTR+TWwNtKwJS6kwCgBWThcIQ6yI1jWEdrJYYFmHXJG/Rf/Nw8twEVh8l/Z0M";

    /*
    final static int ARM_ANGLE_COLLECT_POSITION = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_COLLECT_POSITION = -10000;  //TEST POSITION
    final static int ARM_ANGLE_TRAVEL_POSITION = -5600;
    final static int ARM_ANGLE_SCORE_POSITION = -1000;    //MUST VERIFY
    final static int ARM_ANGLE_DUMP_POSITION = -9500;      //-11000 might be too low, can hit glass

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    static final int NEVEREST_60_CPR = 1680;
    static final int NEVEREST_40_CPR = 1120;
    static final int NEVEREST_20_CPR = 560;

    //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
    final static int ARM_EXTENSION_COLLECTION_POSITION = -56880; //  54946 was observed in -57500;  //test position
    final static int ARM_EXTENSION_TRAVEL_POSITION = -27800;
    final static int ARM_EXTENSION_DUMP_POSITION = -27800;

    final static int LATCH_DEPLOY_POSITION = -13800;        //13900 is a little too high, -13200 usually good
    //13500 is a little high for our practice lander
    final static int LATCH_DRIVE_POSITION = -5900;          //5900 is good, could be a little higher
    final static int LATCH_ENGAGE_POSITION = -11000;
    final static int LATCH_RISEUP_POSITION = -2500;

    //MOTOR PROTECTION ENCODER
    final static int ARM_ANGLE_LIMIT = -11750;  //REAL POSITION
    //final static int ARM_ANGLE_LIMIT = -10000;  //TEST POSITION
    final static int ARM_EXTENSION_LIMIT = -57500;
    final static int LATCH_LIMIT = -14000;


    final static long SAMPLING_DRIVE_TIME = 1350;
    final static long SAMPLING_EXTRA_DRIVE_TIME = 500;
    final static double SAMPLE_ALIGNMENT_TWIST_ANGLE = 30;
    final static long CENTER_SAMPLE_CRATER_DRIVE_ALIGNMENT_TIME = 1200;

    final static long DEPOT_DRIVE_TIME = 1100;
    final static long DEPOT_EXTRA_DRIVE_TIME = 0;
    final static long CRATER_DRIVE_TIME = 2800;  //was 2500 for first successful sample
    final static long EXTRA_CRATER_DRIVE_TIME = 400;
    */
    //****************************************************************
    //END CONSTANTS

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration for gyro
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //reverse direction for opposite side motors
        //This is needed based on how the motors are mounted on the robot
        //Clockwise vs. Counter Clockwise being forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Create an array of motors with their associated Power Setting
        ArrayList<DcMotor> motorList= new ArrayList<>();
        motorList.add(frontLeft);
        motorList.add(frontRight);
        motorList.add(backLeft);
        motorList.add(backRight);

        //Initialize motors for manipulator
        armAngleMotor = hardwareMap.get(DcMotor.class, "armAngleMotor");
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armAngleMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");

        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        latchMotor.setDirection(DcMotor.Direction.REVERSE);
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Initialize the variables that are being used in the main loop
        double spinAngle;    //NOTE:  Positive spin is towards right
        double driveX;
        double driveY;
        double spinSpeed;
        double twistToAngleSpeed = 0.35;
        long delayTime;  //delay time in milliseconds
        long extraDelayTime;  //Based on drive trajectory, sometimes extra time is needed
        double driveComponentForSamplingDirection;
        boolean goldPositionDetermined = false;
        GoldPosition goldPosition = GoldPosition.UNKNOWN;
        //Wait for the game to start(driver presses PLAY)

        // Set up our telemetry dashboard
        composeTelemetry();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        waitForStart();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //This autonomous mode performs the following tasks:
        //  1)  a) Scans from latch
        //      b) Lands
        //      c) Moves away from lander a little
        //      d)Spins to face sampling area
        //  3)  Moves  to sample
        //
        //  4)  Lowers the latch to prepare for reattaching
        //  5)  Twists a little to align to depot
        //  6)  Lower arm angle to drop marker
        //      b) reverse collector motors to drop marker
        //  7) Retract arm angle to drive position
        //  8) If in the center, move back a little to avoid the other sampling objects
        //  9) Turn towards the crater
        //  10) Move to the crater

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  1) Land -->Unlatch from lander

        if (tfod != null) {
            tfod.activate();
        }

        latchMotor.setTargetPosition(R.integer.LATCH_DEPLOY_POSITION);
        latchMotor.setPower(1);

        //Initialize variables for the sampling loop
        int samplingScanCount = 0;
        int goldLeftCount = 0;
        int goldCenterCount = 0;
        int goldRightCount = 0;
        int samplingResponseReceived = 0;
        double samplingInputPercentage = 0;
        double goldLeftConfidence = 0;
        double goldCenterConfidence = 0;
        double goldRightConfidence = 0;
        double thresholdConfidence = 0.5;

        while(latchMotor.isBusy()) {
            //while the robot is descending, scan for minerals
            if (tfod != null) {
                TensorFlowRefactor tensorFlowRefactor = new TensorFlowRefactor().invoke();
                samplingScanCount++;    //Increment the number of samping scans
                //Now number of loops where sampling response is received
                if (tensorFlowRefactor.getGoldPosition() != GoldPosition.UNKNOWN
                        | tensorFlowRefactor.getGoldNotLocated() != GoldPosition.UNKNOWN) {
                    samplingResponseReceived++;
                }
                //Now use the results from the scan to record observations
                if (tensorFlowRefactor.getGoldPosition() == GoldPosition.LEFT) goldLeftCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.CENTER)
                    goldCenterCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.RIGHT)
                    goldRightCount++;

                if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.LEFT) goldLeftCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.CENTER)
                    goldCenterCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.RIGHT)
                    goldRightCount--;

            }
            if (samplingResponseReceived > 0) {
                goldLeftConfidence = (double) (goldLeftCount) / samplingResponseReceived;
                goldCenterConfidence = (double) (goldCenterCount) / samplingResponseReceived;
                goldRightConfidence = (double) (goldRightCount) / samplingResponseReceived;
                samplingInputPercentage = (double) (samplingResponseReceived) / samplingScanCount;

            }
            telemetry.addData("Scan Count", samplingScanCount);
            telemetry.addData("Input Rate", samplingInputPercentage);
            telemetry.addData("Left Rating", goldLeftConfidence);
            telemetry.addData("Center Rating", goldCenterConfidence);
            telemetry.addData("Right Rating", goldRightConfidence);
            telemetry.update();
        }

        //Based on the confidence numbers calculated above, determine the final goldPosition
        if (goldLeftConfidence>goldCenterConfidence
                && goldLeftConfidence > goldRightConfidence
                && goldLeftConfidence > thresholdConfidence){
            goldPosition = GoldPosition.LEFT;
        }else if (goldCenterConfidence>goldLeftConfidence
                && goldCenterConfidence > goldRightConfidence
                && goldCenterConfidence > thresholdConfidence){
            goldPosition = GoldPosition.CENTER;
        }else if(goldRightConfidence>goldLeftConfidence
                && goldRightConfidence > goldCenterConfidence
                && goldRightConfidence>thresholdConfidence){
            goldPosition = GoldPosition.RIGHT;
        } else {goldPosition = GoldPosition.UNKNOWN;}

        //If position was determined, set the boolean flag
        if (goldPosition != GoldPosition.UNKNOWN) goldPositionDetermined = true;

        //TODO:  Add logic to consider positions that were identified as goldNotLocated
        //if gold location is not determined, then randomly assign a location
        if(!goldPositionDetermined){
            double randomNumber = Math.random();
            if(randomNumber <= 0.33){
                goldPosition = GoldPosition.LEFT;
            } else if (randomNumber <=0.67){
                goldPosition = GoldPosition.CENTER;
            } else{
                goldPosition = GoldPosition.RIGHT;
            }
        }
        //Report out the final position
        telemetry.addData("Gold Position", goldPosition.toString());
        telemetry.update();


        //Cut power to the latch motor
        latchMotor.setPower(0);

        //Start the arm extension
        armExtensionMotor.setTargetPosition(R.integer.ARM_EXTENSION_COLLECTION_POSITION);
        armExtensionMotor.setPower(1);

        //Get the arm to travel position
        armAngleMotor.setTargetPosition(R.integer.ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);

        //  b) move away from the lander a little bit
        //Delay a small amount while moving
        delayTime = 300;
        driveX = 1;
        driveY = 0;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  d) Twist the front of the robot to the sampling area
        twistToAngle(85, twistToAngleSpeed, motorList);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  3) Move to sample
        //     Based on the position of the gold element, align to knock it off

        driveY = .85;         //Motion
        delayTime = (long) R.integer.SAMPLING_DRIVE_TIME;
        extraDelayTime = (long) R.integer.SAMPLING_EXTRA_DRIVE_TIME;

        //************************************
        if (goldPosition == GoldPosition.LEFT){
            driveX = -SAMPLING_DRIVE_COMPONENT;         //Motion in y direction to get to left spot
            delayTime += extraDelayTime;        //Add extra drive time because longer distance to spot
        } else if (goldPosition == GoldPosition.RIGHT){
            driveX = SAMPLING_DRIVE_COMPONENT;
            delayTime += extraDelayTime;  //Add extra drive time because longer distance to spot
        } else {
            driveX = 0;
            driveY = 1;
        }
        spinSpeed = 0;      //This is used to determine how to spin the robot
        //This function will perform the drive step and stop the motors
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  4) Lowers the latch to prepare for reattaching
        //      After starting the drive step, move latch down to the DRIVE_POSITION
        latchMotor.setTargetPosition(R.integer.LATCH_DRIVE_POSITION);
        latchMotor.setPower(1);


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  5) Spin a little for better alignment if left or right
        spinAngle = R.integer.SAMPLE_ALIGNMENT_TWIST_ANGLE;
        if (goldPosition == GoldPosition.RIGHT){
            spinAngle *= -1;
        }
        //Only spin if cube is left or right (not center)
        if (goldPosition == GoldPosition.RIGHT | goldPosition == GoldPosition.LEFT){
            twistToAngle(spinAngle,twistToAngleSpeed,motorList);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  6)  Lower arm angle to drop marker
        armAngleMotor.setTargetPosition(R.integer.ARM_ANGLE_DUMP_POSITION);
        armAngleMotor.setPower(1);
        while(armAngleMotor.isBusy()){
            //Wait for the marker to be placed
            //We could add a touch sensor here to improve timing
            telemetry.update();
        }
        armAngleMotor.setPower(0);

        //  b) Spit out the marker
        collectMotor.setPower(-1);
        delayTime = 1000;
        performDriveStep(0, 0, 0, delayTime, motorList);  //delay for a second
        collectMotor.setPower(0);       //turn off the motor

        //  7) Retract arm angle to drive position
        armAngleMotor.setTargetPosition(R.integer.ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);
        telemetry.update();
        //Start raising the arm up to traveling position and then start moving.
        //Delay a small amount while moving
        delayTime = 1000;
        driveX = 0;
        driveY = 0;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  8) If in the center, move forward a little to avoid the other sampling objects
        if(goldPosition == GoldPosition.CENTER) {
            delayTime = (long) (R.integer.SAMPLING_DRIVE_TIME*.7);
            driveX = 0;
            driveY = 1;
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

            delayTime = 400;
            driveX = 0;
            driveY = -1;
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  9) Spin to face the crater
        //  The crater should be oriented 135 degrees to the right of the initialized position (-135 wrt gyro)
        //  To determine spin angle, subtract -135 (aka add 135)


        spinAngle = 120;
        if (goldPosition == GoldPosition.LEFT | goldPosition == GoldPosition.RIGHT) spinAngle += R.integer.SAMPLE_ALIGNMENT_TWIST_ANGLE;
        if (goldPosition == GoldPosition.RIGHT) spinAngle += 5;  //Add a little extra spin based on trials
        if(goldPosition == GoldPosition.LEFT) {
            spinAngle *= -1;
        }

        twistToAngle(spinAngle, twistToAngleSpeed, motorList);

        //turnToFieldHeading(45, motorList);
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  9) If coming from the center, move a little to avoid the lander leg
        if(goldPosition == GoldPosition.CENTER) {
            delayTime = (long) R.integer.CENTER_SAMPLE_CRATER_DRIVE_ALIGNMENT_TIME;
            driveX = -.7;  //Need to go mostly left
            driveY = .4;    //And forward some
            spinSpeed = 0;
            performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement
        }


        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  10) Drive to the crater
        //  If center cube was sampled, add some drive time
        delayTime = (long) R.integer.CRATER_DRIVE_TIME;
        if (goldPosition==GoldPosition.LEFT) delayTime += (long) R.integer.EXTRA_CRATER_DRIVE_TIME;
        driveX = 0;
        driveY = 1;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        //Make sure arm is fully extended and armAngle is in desired location before ending
        while (armAngleMotor.isBusy() | armExtensionMotor.isBusy()){
            //Wait for it to complete
        }
    }


}
