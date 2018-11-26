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
public class Auton_Depot_DEBUG extends eBotsOpMode {

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

    //****************************************************************
    //END CONSTANTS

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration for gyro
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode(){
        //Prepare the gyro in the Expansion hub
/*        initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList= new ArrayList<>();
        initializeDriveMotors(motorList);

        //Initialize motors for manipulator
        initializeManipMotors();*/

        //Initialize the variables that are being used in the main loop
        double spinAngle;    //NOTE:  Positive spin is towards right
        double driveX;
        double driveY;
        double spinSpeed;
        double twistToAngleSpeed = 0.35;
        long delayTime;  //delay time in milliseconds
        long extraDelayTime;  //Based on drive trajectory, sometimes extra time is needed
        boolean goldPositionDetermined = false;
        GoldPosition goldPosition = GoldPosition.UNKNOWN;
        int goldX = -1;
        int silver1X = -1;
        int silver2X = -1;
        //Wait for the game to start(driver presses PLAY)

        // Set up our telemetry dashboard
        //composeTelemetry();

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
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


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
        while(opModeIsActive()){
            //run this loop while the opmode until user pushes stop
            //if (tfod != null && !goldPositionDetermined) {
            if (tfod != null) {
                TensorFlowRefactor tensorFlowRefactor = new TensorFlowRefactor().invoke();
                samplingScanCount++;    //Increment the number of samping scans
                //Now number of loops where sampling response is received
                if(tensorFlowRefactor.getGoldPosition() != GoldPosition.UNKNOWN
                        | tensorFlowRefactor.getGoldNotLocated() != GoldPosition.UNKNOWN){
                    samplingResponseReceived ++;
                }
                //Now use the results from the scan to record observations
                if(tensorFlowRefactor.getGoldPosition() == GoldPosition.LEFT) goldLeftCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.CENTER) goldCenterCount++;
                else if (tensorFlowRefactor.getGoldPosition() == GoldPosition.RIGHT) goldRightCount++;

                if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.LEFT) goldLeftCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.CENTER) goldCenterCount--;
                else if (tensorFlowRefactor.getGoldNotLocated() == GoldPosition.RIGHT) goldRightCount--;

            }
            if(samplingResponseReceived>0){
                goldLeftConfidence = (double)(goldLeftCount) / samplingResponseReceived;
                goldCenterConfidence = (double)(goldCenterCount) / samplingResponseReceived;
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

        //if cube location is not determined, then randomly assign a location
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
    }

}
