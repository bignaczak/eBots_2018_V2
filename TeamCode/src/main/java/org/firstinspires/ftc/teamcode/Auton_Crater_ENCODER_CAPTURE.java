package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.content.ContextWrapper;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Environment;
import android.util.Log;
import android.content.ContextWrapper.*;
import android.view.View;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.ArrayList;

//@Disabled
@Autonomous
public class Auton_Crater_ENCODER_CAPTURE extends eBotsOpMode {
    //For encoder capture, drive encoders are zeroed after each drive step
    //The encoder counts are then written to a file on the phone
    private boolean skipMotorInit=false;
    private boolean loggingActive = true;
    private boolean noRollConsideration = false;
    private boolean yesRollConsideration = true;


    private void executeCraterLeftAuton(ArrayList<DcMotor> motorList,
                                        ArrayList<String> driveSteps,
                                        File file){

        //Overview
        /*push away*/                           double [] sampleLeftMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleLeftTurn0 = -90;
        /*move in front of sample*/             double [] sampleLeftMove1 = new double[] {34, -22, 0};
        //Lower latch
        /*Simple sample move*/                  double [] sampleLeftMove2 = new double[] {0, -2, 0};  //simple sample move
        /*move back a little*/                  double [] sampleLeftMove3 = new double[] {0,4,0};  //pull it back
        /*turn to align with wall*/             double sampleLeftTurn1 = 45;
        /*move toward crater and towards wall*/ double [] sampleLeftMove3_2 = new double[] {10,10,0};  //pull it back

        /*Align with wall*/                     double [] sampleLeftMove4 = new double[] {17,-17,0};
        /*turn for crater drive (overturn)*/    double sampleLeftTurn2 = 55;
        /*turn for crater drive (slight angle)*/double sampleLeftTurn3 = 45;
        //Move arm to dump position
        /*Drive to Depot*/                      double [] sampleLeftMove5 = new double[] {2,2,0};
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position

        /*drive to crater*/                     double [] sampleLeftMove6 = new double[] {-24,-24,0};
        //Note:  facing the wrong way
        //---------------------------------------
        zeroDriveMotorEncoders(motorList);
        //Push off
        moveByDistance(sampleLeftMove0[0],sampleLeftMove0[1],sampleLeftMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleLeftTurn0, motorList);


        //Move to Sample
        moveByDistance(sampleLeftMove1[0],sampleLeftMove1[1],sampleLeftMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Lower the latch
        lowerLatchToDrivePosition();

        // Simple Sample move
        //moveByDistance(sampleLeftMove2[0],sampleLeftMove2[1],sampleLeftMove2[2],motorList,"TimedTranslateAndSpin");

        //Pull back a little
        moveByDistance(sampleLeftMove3[0],sampleLeftMove3[1],sampleLeftMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Turn to Crater
        turnToFieldHeading(sampleLeftTurn1,motorList);

        /*move toward crater and towards wall*/
        moveByDistance(sampleLeftMove3_2[0],sampleLeftMove3_2[1],sampleLeftMove3_2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Snug to wall
        moveByDistance(sampleLeftMove4[0],sampleLeftMove4[1],sampleLeftMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //straighten for drive
        turnToFieldHeading(sampleLeftTurn2,motorList);

        turnToFieldHeading(sampleLeftTurn3,motorList);

        moveArmToDumpPosition();

        //Drive to Depot
        moveByDistance(sampleLeftMove5[0],sampleLeftMove5[1],sampleLeftMove5[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        depositMarkerInDepot(motorList);
        moveArmtoTravelPosition();

        //Drive back to crater
        moveByDistance(sampleLeftMove6[0],sampleLeftMove6[1],sampleLeftMove6[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        waitForArmsToMove();

    }
    private void executeCraterCenterAuton(ArrayList<DcMotor> motorList,
                                          ArrayList<String> driveSteps,
                                          File file){

        //Overview
        /*push away*/                           double [] sampleCenterMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleCenterTurn0 = -90;
        /*sample*/                              double [] sampleCenterMove1 = new double[] {0, -22, 0};
        //Lower latch
        /*move back a little*/                  double [] sampleCenterMove2 = new double[] {0,7,0};  //pull it back
        /*move left to prepare for depot drive*/double [] sampleCenterMove3 = new double[] {30,0,0};
        /*turn to align with wall*/             double sampleCenterTurn1 = 45;
        /*Snug up to wall*/                     double [] sampleCenterMove4 = new double[] {8,-8,0};
        /*turn for crater drive (overturn)*/    double sampleCenterTurn2 = 55;
        /*turn for crater drive (slight angle)*/double sampleCenterTurn3 = 47;
        //Move arm to dump position
        /*Drive to Depot*/                      double [] sampleCenterMove5 = new double[] {15,15,0};  //Drive to Depot
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position

        /*drive to crater*/                     double [] sampleCenterMove6 = new double[] {-27,-27,0};
        //Note:  facing the wrong way
        //---------------------------------------

        zeroDriveMotorEncoders(motorList);

        //Push off
        moveByDistance(sampleCenterMove0[0],sampleCenterMove0[1],sampleCenterMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleCenterTurn0, motorList);


        //Sample
        moveByDistance(sampleCenterMove1[0],sampleCenterMove1[1],sampleCenterMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //Lower the latch
        lowerLatchToDrivePosition();

        //Pull back
        moveByDistance(sampleCenterMove2[0],sampleCenterMove2[1],sampleCenterMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Move left to prepare for depot drive
        moveByDistance(sampleCenterMove3[0],sampleCenterMove3[1],sampleCenterMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Turn to align with wall
        turnToFieldHeading(sampleCenterTurn1,motorList);

        //Snug up to wall
        moveByDistance(sampleCenterMove4[0],sampleCenterMove4[1],sampleCenterMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Align spin with wall
        turnToFieldHeading(sampleCenterTurn2, motorList);
        turnToFieldHeading(sampleCenterTurn3, motorList);

        //prepare to claim
        moveArmToDumpPosition();

        //Drive to Depot
        moveByDistance(sampleCenterMove5[0],sampleCenterMove5[1],sampleCenterMove5[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Turn to align with wall
        turnToFieldHeading(sampleCenterTurn1,motorList);

        //Claim
        depositMarkerInDepot(motorList);
        moveArmtoTravelPosition();

        //Drive to crater
        moveByDistance(sampleCenterMove6[0],sampleCenterMove6[1],sampleCenterMove6[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        waitForArmsToMove();

    }
    private void executeCraterRightAuton(ArrayList<DcMotor> motorList,
                                         ArrayList<String> driveSteps,
                                         File file){
        //Overview
        /*push away*/                           double [] sampleRightMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleRightTurn0 = -90;
        /*sample*/                              double [] sampleRightMove1 = new double[] {-34, -22, 0};
        //Lower latch
        /*move back a little*/                  double [] sampleRightMove2 = new double[] {16,16,0};  //pull it back
        /*move left to prepare for depot drive*/double [] sampleRightMove3 = new double[] {45,0,0};
        /*turn to align with wall*/             double sampleRightTurn1 = 45;
        /*Snug up to wall, take inside route*/  double [] sampleRightMove4 = new double[] {8,-8,0};
        /*turn for depot drive (overturn)*/     double sampleRightTurn2 = 55;
        /*turn for depot drive (slight angle)*/ double sampleRightTurn3 = 43;
        //Move arm to dump position
        /*Drive to Depot*/                      double [] sampleRightMove5 = new double[] {15,15,0};  //Drive to Depot  (slight angle to depot)
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position
        /*back into crater*/          double [] sampleRightMove6 = new double[] {-27,-27,0};
        //Note:  facing the wrong way
        //---------------------------------------


        zeroDriveMotorEncoders(motorList);

        //Push off
        moveByDistance(sampleRightMove0[0],sampleRightMove0[1],sampleRightMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleRightTurn0, motorList);


        //Sample
        moveByDistance(sampleRightMove1[0],sampleRightMove1[1],sampleRightMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //Lower the latch
        lowerLatchToDrivePosition();

        //Pull back
        moveByDistance(sampleRightMove2[0],sampleRightMove2[1],sampleRightMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Move left to prepare for depot drive
        moveByDistance(sampleRightMove3[0],sampleRightMove3[1],sampleRightMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Turn to align with wall
        turnToFieldHeading(sampleRightTurn1,motorList);

        //Snug up to wall
        moveByDistance(sampleRightMove4[0],sampleRightMove4[1],sampleRightMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Don't Align spin with wall
        turnToFieldHeading(sampleRightTurn2, motorList);
        turnToFieldHeading(sampleRightTurn3, motorList);

        //prepare to claim
        moveArmToDumpPosition();

        //Drive to Depot
        moveByDistance(sampleRightMove5[0],sampleRightMove5[1],sampleRightMove5[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //Realign
        turnToFieldHeading(sampleRightTurn3, motorList);

        //Claim
        depositMarkerInDepot(motorList);
        moveArmtoTravelPosition();

        //Back into crater
        moveByDistance(sampleRightMove6[0],sampleRightMove6[1],sampleRightMove6[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        waitForArmsToMove();
    }


    @Override
    public void runOpMode(){

        //Prepare the gyro in the Expansion hub
        if (!skipMotorInit) initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList= new ArrayList<>();
        if (!skipMotorInit) initializeDriveMotors(motorList);

        //Initialize motors for manipulator
        if (!skipMotorInit) initializeManipMotors();

        //Initialize the variables that are being used in the main loop
        GoldPosition goldPosition = GoldPosition.UNKNOWN;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        //Prepare to log the event to file
        File outputFile = initializeDriveStepLogFile();
        writeFileHeader(outputFile, "CRATER side encoder capture");
        String confirmWrite = readFileContents(outputFile.getName());
        ArrayList<String> driveSteps = new ArrayList<>();      //This is the actual driveSteps
        ArrayList<String> driveStepInstructions = new ArrayList<>();    //This is the read-in instructions

        telemetry.addData("File Read Status",
                outputFile.getName() + "--" + confirmWrite);
        telemetry.update();
        //****************END INITIALIZE*******************
        //*************************************************
        //Wait for the game to start(driver presses PLAY)
        waitForStart();
        //*************************************************
        //**************START AUTON ROUTINE****************

        if(skipMotorInit){

            goldPosition = selectRandomGoldPosition();
            writeFileHeader(outputFile, "Gold position: " + goldPosition.toString());

            //Receive the encoder instructions
            String str = "";
            //TODO:  Note that this filename must be updated for the two sides
            String instructionFileName = "CRATER_" + goldPosition.toString();
            getDriveStepInstructions(instructionFileName, driveStepInstructions);
            telemetry.addData("Gold Position", goldPosition.toString());
            for (int i = 0; i<driveStepInstructions.size(); i++){
                telemetry.addData(Integer.toString(i), driveStepInstructions.get(i));
            }
            telemetry.update();
            sleep(3000);


            //Create random drive steps encoder clicks and write values to file
            for(int i=0; i<8;i++){
                writeDriveStepClicksToFile(outputFile, driveSteps, skipMotorInit);
            }
            //Write to telemtry for screen shot
            for (int i=0; i<driveSteps.size(); i++) {
                telemetry.addData(Integer.toString(i), driveSteps.get(i));
            }
            telemetry.update();
            //Allow time for screen shot
            sleep(3000);

            int[] motorEncoderTargets = new int[4];
            parseMotorEncoderTargets(driveStepInstructions.get(0),motorEncoderTargets);
            telemetry.addData("DriveStep", "0");
            telemetry.addData("frontLeft", motorEncoderTargets[0]);
            telemetry.addData("frontRight", motorEncoderTargets[1]);
            telemetry.addData("backLeft", motorEncoderTargets[2]);
            telemetry.addData("backRight", motorEncoderTargets[3]);
            telemetry.update();
            sleep(3000);
        }else{
            //TODO:  FIX THE FIELD OF VIEW
            goldPosition=landAndLocateGoldMineral();

            writeFileHeader(outputFile, "Gold position: " + goldPosition.toString());

            if(goldPosition == GoldPosition.LEFT){
                executeCraterLeftAuton(motorList, driveSteps, outputFile);
            }else if(goldPosition == GoldPosition.RIGHT){
                executeCraterRightAuton(motorList, driveSteps, outputFile);
            }else  {        //CENTER or UNKNOWN
                executeCraterCenterAuton(motorList, driveSteps, outputFile);
            }
        }

    }


    //    public  boolean isStoragePermissionGranted() {
//        String TAG = "Helper";
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
//            if (checkSelfPermission(android.Manifest.permission.WRITE_EXTERNAL_STORAGE)
//                    == PackageManager.PERMISSION_GRANTED) {
//                Log.v(TAG,"Permission is granted");
//                return true;
//            } else {
//
//                Log.v(TAG,"Permission is revoked");
//                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1);
//                return false;
//            }
//        }
//        else { //permission is automatically granted on sdk<23 upon installation
//            Log.v(TAG,"Permission is granted");
//            return true;
//        }
//    }

}
