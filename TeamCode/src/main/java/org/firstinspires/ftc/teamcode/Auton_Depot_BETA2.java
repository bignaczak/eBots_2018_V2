package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

@Disabled
@Autonomous
public class Auton_Depot_BETA2 extends eBotsOpMode {

    //This autonomous mode performs a sequence of tasks intended to :
    // move rover from the depot latch on the lander
    // sample
    // claim depot
    // park in crater
    // detailed list of operations is listed in opMode main body


    //  DEFINE CONSTANTS FOR THE ROBOT
    //  THESE ARE ALL POSITIONS ASSUMED THE ROBOT IS COMPLETELY FOLDED PRIOR TO START OF AUTON


    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration for gyro
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode(){
        //Prepare the gyro in the Expansion hub
        initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList= new ArrayList<>();
        initializeDriveMotors(motorList);

        //Initialize motors for manipulator
        initializeManipMotors();

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

        latchMotor.setTargetPosition(LATCH_DEPLOY_POSITION);
        telemetry.addData("Latch Target", latchMotor.getTargetPosition());
        telemetry.addData("Latch Current Position", latchMotor.getCurrentPosition());
        telemetry.update();
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
            telemetry.addData("Latch Target", latchMotor.getTargetPosition());
            telemetry.addData("Latch Current Position", latchMotor.getCurrentPosition());
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

        /*//Start the arm extension
        armExtensionMotor.setTargetPosition(R.integer.ARM_EXTENSION_COLLECTION_POSITION);
        armExtensionMotor.setPower(1);

        //Get the arm to travel position
        armAngleMotor.setTargetPosition(R.integer.ARM_ANGLE_TRAVEL_POSITION);
        armAngleMotor.setPower(1);
*/
        //  b) move away from the lander a little bit
        //Delay a small amount while moving
        delayTime = 300;
        driveX = 1;
        driveY = 0;
        spinSpeed = 0;
        performDriveStep(driveX, driveY, spinSpeed, delayTime, motorList);  //Just a pause, no movement

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //  d) Twist the front of the robot to the sampling area
        //twistToAngle(85, twistToAngleSpeed, motorList);
        boolean hasTurnedSuccessfully = turnToFieldHeading(-90, motorList);

        latchMotor.setTargetPosition(-50);
        latchMotor.setPower(1);
        while(latchMotor.isBusy()){
            //just wait
            getCurrentHeading();
            telemetry.addData("heading", currentHeading);
            telemetry.addData("turn successful", hasTurnedSuccessfully);
            telemetry.update();
        }


    }


}
