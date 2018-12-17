package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.io.BufferedReader;
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
public class Auton_Depot extends eBotsOpMode {

    private boolean skipMotorInit=false;
    private boolean loggingActive = true;
    private boolean noRollConsideration = false;
    private boolean yesRollConsideration = true;

    private void executeDepotLeftAuton(ArrayList<DcMotor> motorList,
                                       ArrayList<String> driveSteps,
                                       File file){
        //Overview
        /*push away*/                           double [] sampleLeftMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face depot*/                  double sampleLeftTurn0 = -90;
        /*move in front of sample*/             double [] sampleLeftMove1 = new double[] {25, -22, 0};
        //Lower latch and move arm to dump position
        /*push gold into depot*/                double [] sampleLeftMove2 = new double[] {1, -35, -(Math.PI*1)/6};
        /*turn to align with wall*/             double sampleLeftTurn1 = -135;
        /*move back and towards wall little*/   double [] sampleLeftMove3 = new double[] {2, 2, 0};
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position
        /*turn for crater drive*/               double sampleLeftTurn2 = 135;
        /*must turn to right, so 2 steps */     double sampleLeftTurn2_2 = 45;
        /*Align with wall*/                     double [] sampleLeftMove4 = new double[] {.5,-0.5,0};
        /*straighten for drive (Overturn)*/     double sampleLeftTurn3 = 55;
        /*straighten for drive slight angle*/   double sampleLeftTurn4 = 48;
        /*drive to crater*/                     double [] sampleLeftMove5 = new double[] {20,20,0};
        /*straighten for drive slight angle*/   double sampleLeftTurn5 = 45;
        /*drive to crater*/                     double [] sampleLeftMove6 = new double[] {10,10,0};

        zeroDriveMotorEncoders(motorList);

        //Push off
        moveByDistance(sampleLeftMove0[0],sampleLeftMove0[1],sampleLeftMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        moveArmtoTravelPosition();
        extendArm();

        turnToFieldHeading(sampleLeftTurn0, motorList);

        //Move in front of Sample
        moveByDistance(sampleLeftMove1[0],sampleLeftMove1[1],sampleLeftMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push sample to depot
        moveByDistance(sampleLeftMove2[0],sampleLeftMove2[1],sampleLeftMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Turn to align with wall
        turnToFieldHeading(sampleLeftTurn1, motorList);

        //move back and towards wall little
        moveByDistance(sampleLeftMove3[0],sampleLeftMove3[1],sampleLeftMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //wait for claiming
        depositMarkerInDepot(motorList);

        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();

        //turn for crater drive, 2 steps because must dictate direction
        turnToFieldHeading(sampleLeftTurn2, motorList);
        turnToFieldHeading(sampleLeftTurn2_2, motorList);

        //push against wall
        moveByDistance(sampleLeftMove4[0],sampleLeftMove4[1],sampleLeftMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //straighten for drive
        turnToFieldHeading(sampleLeftTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleLeftTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleLeftMove5[0],sampleLeftMove5[1],sampleLeftMove5[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        turnToFieldHeading(sampleLeftTurn5, motorList);  //now align for the drive
        moveByDistance(sampleLeftMove6[0],sampleLeftMove6[1],sampleLeftMove6[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        waitForArmsToMove();

    }
    private void executeDepotCenterAuton(ArrayList<DcMotor> motorList,
                                         ArrayList<String> driveSteps,
                                         File file){

        /*push away*/                           double [] sampleCenterMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face depot*/                  double sampleCenterTurn0 = -90;
        /*move in front of sample*/             double [] sampleCenterMove1 = new double[] {0, -20, 0};
        //Deposit marker and claim depot
        //Lower latch and move arm to travel position
        /*push gold into depot*/                double [] sampleCenterMove2 = new double[] {0, -18, 0};

        /*move back and towards wall little*/   double [] sampleCenterMove3 = new double[] {0, 3, 0};//Move back
        /*turn to align with wall*/             double sampleCenterTurn1 = 45;
        /*Align with wall*/                     double [] sampleCenterMove4 = new double[] {6,-6,0};//Align to wall
        /*straighten for drive (Overturn)*/     double sampleCenterTurn3 = 55;
        /*straighten for drive slight angle*/   double sampleCenterTurn4 =47;
        /*drive to crater*/                     double [] sampleCenterMove5 = new double[] {30,30,0};
        /*drive to crater*/                     double [] sampleCenterMove5_2 = new double[] {12,12,0};

        //--------------------------------------------
        zeroDriveMotorEncoders(motorList);

        //Move to Sample
        moveByDistance(sampleCenterMove0[0],sampleCenterMove0[1],sampleCenterMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps, skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        moveArmtoTravelPosition();
        extendArm();

        //Turn to depot
        turnToFieldHeading(sampleCenterTurn0,motorList);

        //Move to Sample
        moveByDistance(sampleCenterMove1[0],sampleCenterMove1[1],sampleCenterMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //wait for claiming
        depositMarkerInDepot(motorList);
        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleCenterMove2[0],sampleCenterMove2[1],sampleCenterMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //move back a little
        moveByDistance(sampleCenterMove3[0],sampleCenterMove3[1],sampleCenterMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //turn towards crater
        turnToFieldHeading(sampleCenterTurn1, motorList);

        //align with wall
        moveByDistance(sampleCenterMove4[0],sampleCenterMove4[1],sampleCenterMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //straighten for drive
        turnToFieldHeading(sampleCenterTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleCenterTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleCenterMove5[0],sampleCenterMove5[1],sampleCenterMove5[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        turnToFieldHeading(sampleCenterTurn4, motorList);  //now align for the drive

        moveByDistance(sampleCenterMove5_2[0],sampleCenterMove5_2[1],sampleCenterMove5_2[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        waitForArmsToMove();

    }
    private void executeDepotRightAuton(ArrayList<DcMotor> motorList,
                                        ArrayList<String> driveSteps,
                                        File file){
        //Overview
        /*push away*/                           double [] sampleRightMove0 = new double[] {0, -3, 0};  //push away
        //Extend arm & move armAngle travel position
        /*turn to face depot*/                  double sampleRightTurn0 = -90;
        /*move in front of sample*/             double [] sampleRightMove1 = new double[] {-35, -20, 0};
        //Lower latch and move arm to dump position
        /*push gold into depot*/                double [] sampleRightMove2 = new double[] {6, -35, (Math.PI*1)/6};
        /*turn to align with wall*/             double sampleRightTurn1 = -45;
        /*move back and towards wall little*/   double [] sampleRightMove3 = new double[] {-10, 10, 0};
        //Deposit marker and claim depot
        //lower latch and move to armAngle to travel position
        /*turn for crater drive--Part1*/        double sampleRightTurn2 = 45;
        /*turn for crater drive--Part2*/        double sampleRightTurn2_2 = 135;
        /*Align with wall*/                     double [] sampleRightMove4 = new double[] {-.5,-0.5,0};
        /*straighten for drive (Overturn)*/     double sampleRightTurn3 = 125;
        /*straighten for drive slight angle*/   double sampleRightTurn4 = 133;
        /*drive to crater*/                     double [] sampleRightMove5 = new double[] {-35,35,0};




        //------------------------------------------------------
        zeroDriveMotorEncoders(motorList);


        //push away
        moveByDistance(sampleRightMove0[0],sampleRightMove0[1],sampleRightMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        moveArmtoTravelPosition();
        extendArm();

        turnToFieldHeading(sampleRightTurn0,motorList);

        //Move to Sample
        moveByDistance(sampleRightMove1[0],sampleRightMove1[1],sampleRightMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);


        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleRightMove2[0],sampleRightMove2[1],sampleRightMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //Align for turn back
        turnToFieldHeading(sampleRightTurn1, motorList);

        //move back a little
        moveByDistance(sampleRightMove3[0],sampleRightMove3[1],sampleRightMove3[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //wait for claiming
        depositMarkerInDepot(motorList);

        //Lower latch
        lowerLatchToDrivePosition();

        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //turn towards crater  --2 parts to ensure direction
        turnToFieldHeading(sampleRightTurn2, motorList);
        turnToFieldHeading(sampleRightTurn2_2, motorList);

        //align with wall
        moveByDistance(sampleRightMove4[0],sampleRightMove4[1],sampleRightMove4[2],motorList,"TimedTranslateAndSpin", noRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
        zeroDriveMotorEncoders(motorList);

        //straighten for drive
        turnToFieldHeading(sampleRightTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleRightTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleRightMove5[0],sampleRightMove5[1],sampleRightMove5[2],motorList,"TimedTranslateAndSpin", yesRollConsideration);
        writeDriveStepClicksToFile(file, driveSteps,skipMotorInit);
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

        //Prepare to log the event to file
        File outputFile = initializeDriveStepLogFile();
        writeFileHeader(outputFile, "DEPOT side encoder capture");
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
            String instructionFileName = "DEPOT_" + goldPosition.toString();
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
            for (int i=0; i<driveSteps.size(); i++) {
                telemetry.addData(Integer.toString(i), driveSteps.get(i));
            }
            telemetry.update();
            //Allow time for screen shot
            sleep(3000);


        }else {
            goldPosition = landAndLocateGoldMineral();

            if (goldPosition == GoldPosition.LEFT) {
                executeDepotLeftAuton(motorList, driveSteps, outputFile);
            } else if (goldPosition == GoldPosition.RIGHT) {
                executeDepotRightAuton(motorList, driveSteps, outputFile);
            } else {        //CENTER or UNKNOWN
                executeDepotCenterAuton(motorList, driveSteps, outputFile);
            }
        }

    }

//    public File initializeDriveStepLogFile() {
//        //This check verifies that storage permissions are granted
//        File newFile = null;
//        if (isExternalStorageWritable()) {
//            Log.d("Helper", "Yes, it is writable");
//
////            if (isStoragePermissionGranted()) {
////                Log.d("Helper", "Looking good now for permissions");
////            } else {
////                Log.d("Helper", "Still got a problem with permissions");
////
////            }
//
//            //Generate a filename using timestamp
//            Timestamp timeStamp = new Timestamp(System.currentTimeMillis());
//            Log.d("Helper", sdf.format(timeStamp));
//            String fileName = sdf.format(timeStamp);
//            newFile = getPublicFileStorageDir(fileName);
//        } else {
//            Log.d("Helper", "No, not writable");
//        }
//
//        //Note:  this may return null
//        return newFile;
//    }
//
//    public void writeDriveStepClicksToFile(File newFile, ArrayList<String> driveSteps){
//
//        try{
//            DriveStep driveStep = new DriveStep();
//            writeToFile(driveStep.toString(), newFile);
//            driveSteps.add(driveStep.toString());
//
//        } catch (IOException e){
//            Log.d ("Helper", "Error writing driveStep");
//            Log.d("Helper", e.getMessage());
//        }
//    }
//
//    private static final SimpleDateFormat sdf = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss");
//
//    public boolean isExternalStorageWritable() {
//        String state = Environment.getExternalStorageState();
//        if (Environment.MEDIA_MOUNTED.equals(state)) {
//            return true;
//        }
//        return false;
//    }
//
//    public File getPublicFileStorageDir(String fileName) {
//        // Get the directory for the user's public pictures directory.
//
//        File directory = new File(Environment.getExternalStoragePublicDirectory(
//                Environment.DIRECTORY_DOCUMENTS),"");
//        Log.d("Helper", directory.getPath());
//        if (!directory.exists()) {
//            directory.mkdirs();
//        }
//        if (!directory.mkdirs()) {
//            Log.e("Helper", "Directory not created");
//        }
//
//        File writeFile = new File(directory.getAbsolutePath() + "/" + fileName);
//        return writeFile;
//    }
//

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

//    public void writeToFile( String textLine, File file ) throws IOException {
//        FileWriter write = new FileWriter( file.getAbsolutePath(), true);
//        PrintWriter printLine = new PrintWriter( write );
//        printLine.printf("%s" + "%n" , textLine);
//        printLine.close();
//    }
//
//    public String readFileContents(String fileName) {
//        String readLine="";
//        try {
//            File myDir = new File(Environment.getExternalStoragePublicDirectory(
//                    Environment.DIRECTORY_DOCUMENTS),"");
//            BufferedReader br = new BufferedReader(new FileReader(myDir + "/"+fileName));
//            readLine = br.readLine();
//
//            // Set TextView text here using tv.setText(s);
//
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        return readLine;
//    }
//
//    public class DriveStep{
//
//        private int flEncoderCounts;
//        private int frEncoderCounts;
//        private int blEncoderCounts;
//        private int brEncoderCounts;
//
//        ArrayList<Integer> encoderArray = new ArrayList<>();
//
//
//        public DriveStep(){
//            this.flEncoderCounts=frontLeft.getCurrentPosition();
//            this.frEncoderCounts=frontRight.getCurrentPosition();
//            this.blEncoderCounts =backLeft.getCurrentPosition();
//            this.brEncoderCounts= backRight.getCurrentPosition();
//            encoderArray.add(flEncoderCounts);
//            encoderArray.add(frEncoderCounts);
//            encoderArray.add(blEncoderCounts);
//            encoderArray.add(brEncoderCounts);
//        }
//        //*************************************
//        //The getters
//
//        public int getFlEncoderCounts() {
//            return flEncoderCounts;
//        }
//
//        public int getFrEncoderCounts() {
//            return frEncoderCounts;
//        }
//
//        public int getBlEncoderCounts() {
//            return blEncoderCounts;
//        }
//
//        public int getBrEncoderCounts() {
//            return brEncoderCounts;
//        }
//
//        //*************************************
//        //The setters
//        public void setFlEncoderCounts(int flEncoderCounts) {
//            this.flEncoderCounts = flEncoderCounts;
//        }
//
//        public void setBlEncoderCounts(int blEncoderCounts) {
//            this.blEncoderCounts = blEncoderCounts;
//        }
//
//        @Override
//        public String toString(){
//            String returnString = "[";
//            for(int i=0; i<encoderArray.size();i++){
//                returnString = returnString + Integer.toString(encoderArray.get(i));
//                if (i != encoderArray.size()-1) returnString = returnString + ", ";
//            }
//            returnString+= "]";
//            return returnString;
//        }
//
//        public void setFrEncoderCounts(int frEncoderCounts) {
//            this.frEncoderCounts = frEncoderCounts;
//        }
//
//        public void setBrEncoderCounts(int brEncoderCounts) {
//            this.brEncoderCounts = brEncoderCounts;
//        }
//
//        private int getRandomEncoderValue(){
//            return (int) (Math.random()*2000);
//        }
//    }
//
//    private void zeroDriveMotorEncoders(ArrayList<DcMotor> motors) {
//        for (DcMotor m:motors){
//            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//    }

}
