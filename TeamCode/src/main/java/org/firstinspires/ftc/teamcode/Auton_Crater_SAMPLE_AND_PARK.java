package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class Auton_Crater_SAMPLE_AND_PARK extends eBotsOpMode {

    private boolean noRollConsideration = false;
    private boolean yesRollConsideration = true;

    private void executeCraterLeftAuton(ArrayList<DcMotor> motorList){

        //Overview
        /*push away*/                           double [] sampleLeftMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleLeftTurn0 = -90;
        /*move in front of sample*/             double [] sampleLeftMove1 = new double[] {20, -20, 0};
        //Lower latch
        /*Simple sample move*/                  double [] sampleLeftMove2 = new double[] {0, -2, 0};  //simple sample move


        //******************ALL ELSE COMMENTED OUT

        /*move back a little*/                  double [] sampleLeftMove3 = new double[] {0,4,0};  //pull it back
        /*turn to align with wall*/             double sampleLeftTurn1 = 45;
        /*move toward crater and towards wall*/ double [] sampleLeftMove3_2 = new double[] {12,12,0};  //pull it back

        /*Align with wall*/                     double [] sampleLeftMove4 = new double[] {17,-17,0};
        /*turn for crater drive (overturn)*/    double sampleLeftTurn2 = 55;
        /*turn for crater drive (slight angle)*/double sampleLeftTurn3 = 45;
        //Move arm to dump position
        /*Drive to Depot*/                      double [] sampleLeftMove5 = new double[] {5,5,0};
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position

        /*drive to crater*/                     double [] sampleLeftMove6 = new double[] {-27,-27,0};
        //Note:  facing the wrong way
        //---------------------------------------

        //Push off
        moveByDistance(sampleLeftMove0[0],sampleLeftMove0[1],sampleLeftMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleLeftTurn0, motorList);


        //Move to Sample
        moveByDistance(sampleLeftMove1[0],sampleLeftMove1[1],sampleLeftMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        //Lower the latch
        lowerLatchToDrivePosition();

        // Simple Sample move
        moveByDistance(sampleLeftMove2[0],sampleLeftMove2[1],sampleLeftMove2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        waitForArmsToMove();


//        //Pull back a little
//        moveByDistance(sampleLeftMove3[0],sampleLeftMove3[1],sampleLeftMove3[2],motorList,"TimedTranslateAndSpin");
//
//        //Turn to Crater
//        turnToFieldHeading(sampleLeftTurn1,motorList);
//
//        /*move toward crater and towards wall*/
//        moveByDistance(sampleLeftMove3_2[0],sampleLeftMove3_2[1],sampleLeftMove3_2[2],motorList,"TimedTranslateAndSpin");
//
//        //Snug to wall
//        moveByDistance(sampleLeftMove4[0],sampleLeftMove4[1],sampleLeftMove4[2],motorList,"TimedTranslateAndSpin");
//
//        //straighten for drive
//        turnToFieldHeading(sampleLeftTurn2,motorList);
//        turnToFieldHeading(sampleLeftTurn3,motorList);
//
//        moveArmToDumpPosition();
//
//        //Drive to Depot
//        moveByDistance(sampleLeftMove5[0],sampleLeftMove5[1],sampleLeftMove5[2],motorList,"TimedTranslateAndSpin");
//
//
//        depositMarkerInDepot(motorList);
//        moveArmtoTravelPosition();
//
//        //Drive back to crater
//        moveByDistance(sampleLeftMove6[0],sampleLeftMove6[1],sampleLeftMove6[2],motorList,"TimedTranslateAndSpin");


    }
    private void executeCraterCenterAuton(ArrayList<DcMotor> motorList){

        //Overview
        /*push away*/                           double [] sampleCenterMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleCenterTurn0 = -90;
        /*sample*/                              double [] sampleCenterMove1 = new double[] {0, -25, 0};
        //Lower latch
        //******************ALL ELSE COMMENTED OUT

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

        /*drive to crater*/                     double [] sampleCenterMove6 = new double[] {-33,-33,0};
        //Note:  facing the wrong way
        //---------------------------------------

        //Push off
        moveByDistance(sampleCenterMove0[0],sampleCenterMove0[1],sampleCenterMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleCenterTurn0, motorList);


        //Sample
        moveByDistance(sampleCenterMove1[0],sampleCenterMove1[1],sampleCenterMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);


        //Lower the latch
        lowerLatchToDrivePosition();

        waitForArmsToMove();


//        //Pull back
//        moveByDistance(sampleCenterMove2[0],sampleCenterMove2[1],sampleCenterMove2[2],motorList,"TimedTranslateAndSpin");
//
//        //Move left to prepare for depot drive
//        moveByDistance(sampleCenterMove3[0],sampleCenterMove3[1],sampleCenterMove3[2],motorList,"TimedTranslateAndSpin");
//
//        //Turn to align with wall
//        turnToFieldHeading(sampleCenterTurn1,motorList);
//
//        //Snug up to wall
//        moveByDistance(sampleCenterMove4[0],sampleCenterMove4[1],sampleCenterMove4[2],motorList,"TimedTranslateAndSpin");
//
//        //Align spin with wall
//        turnToFieldHeading(sampleCenterTurn2, motorList);
//        turnToFieldHeading(sampleCenterTurn3, motorList);
//
//        //prepare to claim
//        moveArmToDumpPosition();
//
//        //Drive to Depot
//        moveByDistance(sampleCenterMove5[0],sampleCenterMove5[1],sampleCenterMove5[2],motorList,"TimedTranslateAndSpin");
//
//        //Turn to align with wall
//        turnToFieldHeading(sampleCenterTurn1,motorList);
//
//        //Claim
//        depositMarkerInDepot(motorList);
//        moveArmtoTravelPosition();
//
//        //Drive to crater
//        moveByDistance(sampleCenterMove6[0],sampleCenterMove6[1],sampleCenterMove6[2],motorList,"TimedTranslateAndSpin");



    }
    private void executeCraterRightAuton(ArrayList<DcMotor> motorList){
        //Overview
        /*push away*/                           double [] sampleRightMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face crater*/                 double sampleRightTurn0 = -90;
        /*sample*/                              double [] sampleRightMove1 = new double[] {-35, -20, 0};
        /*Simple sample move*/                  double [] sampleRightMove1_2 = new double[] {0, -2, 0};  //simple sample move

        //******************ALL ELSE COMMENTED OUT
        //Lower latch
        /*move back a little*/                  double [] sampleRightMove2 = new double[] {17,17,0};  //pull it back
        /*move left to prepare for depot drive*/double [] sampleRightMove3 = new double[] {45,0,0};
        /*turn to align with wall*/             double sampleRightTurn1 = 45;
        /*Snug up to wall, take inside route*/  double [] sampleRightMove4 = new double[] {8,-8,0};
        /*turn for depot drive (overturn)*/     double sampleRightTurn2 = 55;
        /*turn for depot drive (slight angle)*/ double sampleRightTurn3 = 45;
        //Move arm to dump position
        /*Drive to Depot*/                      double [] sampleRightMove5 = new double[] {15,15,0};  //Drive to Depot  (slight angle to depot)
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position
        /*back into crater*/          double [] sampleRightMove6 = new double[] {-30,-30,0};
        //Note:  facing the wrong way
        //---------------------------------------

        //Push off
        moveByDistance(sampleRightMove0[0],sampleRightMove0[1],sampleRightMove0[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        //Unfold and extend arm
        moveArmtoTravelPosition();
        extendArm();

        //Turn to the crater
        turnToFieldHeading(sampleRightTurn0, motorList);


        //Sample
        moveByDistance(sampleRightMove1[0],sampleRightMove1[1],sampleRightMove1[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        //simple Sample
        moveByDistance(sampleRightMove1_2[0],sampleRightMove1_2[1],sampleRightMove1_2[2],motorList,"TimedTranslateAndSpin", noRollConsideration);

        waitForArmsToMove();

//        //Lower the latch
//        lowerLatchToDrivePosition();
//
//        //Pull back
//        moveByDistance(sampleRightMove2[0],sampleRightMove2[1],sampleRightMove2[2],motorList,"TimedTranslateAndSpin");
//
//        //Move left to prepare for depot drive
//        moveByDistance(sampleRightMove3[0],sampleRightMove3[1],sampleRightMove3[2],motorList,"TimedTranslateAndSpin");
//
//        //Turn to align with wall
//        turnToFieldHeading(sampleRightTurn1,motorList);
//
//        //Don't Snug up to wall, take inside route
//        moveByDistance(sampleRightMove4[0],sampleRightMove4[1],sampleRightMove4[2],motorList,"TimedTranslateAndSpin");
//
//        //Don't Align spin with wall
//        turnToFieldHeading(sampleRightTurn2, motorList);
//        turnToFieldHeading(sampleRightTurn3, motorList);
//
//        //prepare to claim
//        moveArmToDumpPosition();
//
//        //Drive to Depot
//        moveByDistance(sampleRightMove5[0],sampleRightMove5[1],sampleRightMove5[2],motorList,"TimedTranslateAndSpin");
//        //Realign
//        turnToFieldHeading(sampleRightTurn3, motorList);
//
//        //Claim
//        depositMarkerInDepot(motorList);
//        moveArmtoTravelPosition();
//
//        //Back into crater
//        moveByDistance(sampleRightMove6[0],sampleRightMove6[1],sampleRightMove6[2],motorList,"TimedTranslateAndSpin");

        waitForArmsToMove();
    }


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


        waitForStart();

        //TODO:  FIX THE FIELD OF VIEW
        goldPosition=landAndLocateGoldMineral();

        if(goldPosition == GoldPosition.LEFT){
            executeCraterLeftAuton(motorList);
        }else if(goldPosition == GoldPosition.RIGHT){
            executeCraterRightAuton(motorList);
        }else  {        //CENTER or UNKNOWN
            executeCraterCenterAuton(motorList);
        }

    }
}
