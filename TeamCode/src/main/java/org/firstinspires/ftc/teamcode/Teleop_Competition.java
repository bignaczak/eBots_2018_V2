package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;
//import java.lang.String;

@TeleOp
public class Teleop_Competition extends LinearOpMode {
    //private Gyroscope imu;
    //Drive Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Manipulation Motors
    private DcMotor armAngleMotor;
    private DcMotor armExtensionMotor;
    private DcMotor collectMotor;
    private DcMotor latchMotor;

    //These are constants used to define counts per revolution of NEVEREST motors with encoders
    private static final int NEVEREST_60_CPR = 1680;
    private static final int NEVEREST_40_CPR = 1120;
    private static final int NEVEREST_20_CPR = 560;
    //private DcMotor left_drive;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    //private ColorSensor sensorColorRange;
    //private CRServo servoTest;


    private void calculateDriveVector(double driveMagnitude, double robotAngle, double spin, double[] outputArray){
        //Create an array of the drive values
        //[Element] --> Wheel
        //  [0] --> Front Left
        //  [1] --> Front Right
        //  [2] --> Back Left
        //  [3] --> Back Right
        outputArray[0] = driveMagnitude * Math.cos(robotAngle) + spin;
        outputArray[1] = driveMagnitude * Math.sin(robotAngle) - spin;
        outputArray[2] = driveMagnitude * Math.sin(robotAngle) + spin;
        outputArray[3] = driveMagnitude * Math.cos(robotAngle) - spin;

        //Now capture the max drive value from the array
        double maxValue = findMaxAbsValue(outputArray);

        //If any of the values exceed 1, then all drive values must be scaled
        //Divide all drive values by the max value to achieve a new max value of 1
        if (maxValue > 1) scaleDrive(1/maxValue, outputArray);
    }

    private void scaleDrive (double scaleFactor, double[] driveArray){
        for (int i=0; i<driveArray.length; i++) {
            driveArray[i] *= scaleFactor;
        }
    }

    private double findMaxAbsValue(double[] array) {
        //Now capture the max drive value from the array
        double maxValue = Math.abs(array[0]);
        for (int i = 1; i < array.length; i++){
            if(Math.abs(array[i]) > maxValue) maxValue = Math.abs(array[i]);
        }
        return maxValue;
    }
    @Override
    public void runOpMode(){
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        //reverse direction for opposite side motors
        //This is needed based on how the motors are mounted on the robot
        //Clockwise vs. Counter Clockwise being forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Set Motor Power Settings");
        telemetry.update();

        //Create an array of motors with their associated Power Setting
        List<DcMotor> motorList= new ArrayList<>();
        motorList.add(frontLeft);
        motorList.add(frontRight);
        motorList.add(backLeft);
        motorList.add(backRight);


        telemetry.addData("Status", "Motor Power Set, Ready");
        telemetry.update();

        //Initialize the variables that are being used in the main loop
        double spin;
        double driveX;
        double driveY;
        double r;       //length of radius for driveX and driveY
        double robotAngle;      //adjust of angle to account for mecanum drive
        double fineAdjust;
        final double fineAdjustThreshold = 0.3;    //Avoid trivial amounts of speed reduction with threshold value
        final double fineAdjustMaxReduction = 0.5; //Don't allow drive to be fully negated
        boolean fineAdjustOn = false;               //Flag if fine adjust is activated
        boolean speedBoostOn = false;               //Maximize motor drive speeds if pressed
        double[] driveValues = new double[4];
        double maxValue;

        //Initialize motors for manipulator
        armAngleMotor = hardwareMap.get(DcMotor.class, "armAngleMotor");
        armAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armAngleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armAngleMotor.setDirection(DcMotor.Direction.REVERSE);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "armExtensionMotor");
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectMotor = hardwareMap.get(DcMotor.class, "collectMotor");

        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int countsPerRevolution = NEVEREST_20_CPR;  //Output counts per revolution of Output Shaft (cpr): 1120
        double degreesToEncoderCounts = countsPerRevolution/360.0;
        int targetAngle;
        int targetEncoderCounts;
        double rightTrigger;
        double leftTrigger;
        double latchDrive;
        double armAngleDrive;
        final double motorThreshold=0.25;
        boolean encoderSafetyOverride = false;
        double spinScaleFactor = 0.5;


        //These are variables to set the motors to preset positions
        //This assumes that all encoders are zeroed at fully retracted position


        //final int ARM_ANGLE_COLLECT_POSITION = -12980;

        final int ARM_ANGLE_COLLECT_POSITION = -6000;  //TEST POSITIION
        final int ARM_ANGLE_TRAVEL_POSITION = -5600;
        final int ARM_ANGLE_DUMP_POSITION = -5600;

        //final int ARM_EXTENSION_COLLECTION_POSITION = 27800;
        final int ARM_EXTENSION_COLLECTION_POSITION = 15000;  //test position
        final int ARM_EXTENSION_TRAVEL_POSITIION = 27800;
        final int ARM_EXTENSION_DUMP_POSITION = 27800;

        final int LATCH_DEPLOY_POSITION = 13900;
        final int LATCH_DRIVE_POSITION = 5900;
        final int LATCH_ENGAGE_POSITION = 9892;
        final int LATCH_RISEUP_POSITION = 1540;


        //MOTOR PROTECTION ENCODER
        final int ARM_ANGLE_LIMIT = -13000;
        final int ARM_EXTENSION_LIMIT = 27850;
        final int LATCH_LIMIT = 14000;

        //Wait for the game to start(driver presses PLAY)
        waitForStart();


        //run until the end of the match (driver presses STOP)
        while(opModeIsActive()){


            //GAMEPAD1 INPUTS
            //----------------------------------------
            //Get the drive inputs from the controller
            //  [LEFT STICK]   --> Direction and Speed
            //  [RIGHT STICK]  --> X Direction dictates spin rate to rotate about robot center
            //  [LEFT TRIGGER] --> Variable reduction in robot speed to allow for fine position adjustment
            //  [RIGHT BUMPER] --> Speed boost, maximized motor drive speed

            driveX = gamepad1.left_stick_x;        //Motion
            driveY = -gamepad1.left_stick_y;         //Motion
            spin = gamepad1.right_stick_x * spinScaleFactor; //This is used to determine how to spin the robot
            fineAdjust = gamepad1.left_trigger;     //Pull to slow motion
            speedBoostOn = gamepad1.right_bumper;   //Push to maximize motor drives

            //r gives the left stick's offset from 0 position by calculating hypotenuse of x and y offset
            r = Math.hypot(driveX, driveY );

            //Robot angle calculates the angle (in radians) and then subtracts pi/4 (45 degrees) from it
            //The 45 degree shift aligns the mecanum vectors for drive
            robotAngle = Math.atan2(driveY, driveX) - Math.PI / 4;
            calculateDriveVector(r, robotAngle, spin, driveValues);     //Calculate motor drive speeds

            //Now allow for fine maneuvering by allowing a slow mode when pushing trigger
            //Trigger is an analog input between 0-1, so it allows for variable adjustment of speed
            //Now scale the drive values based on the level of the trigger
            //We don't want to trigger to allow the joystick to be completely negated
            //And we don't want trivial amounts of speed reduction
            //Initialized variable above Set threshold value to ~0.2 (fineAdjustThreshold)
            // and only allow 80% reduction of speed (fineAdjustMaxReduction)
            if (fineAdjust >= fineAdjustThreshold) {
                fineAdjustOn = true;
                fineAdjust *= fineAdjustMaxReduction;
            }
            else{
                fineAdjustOn = false;
                fineAdjust=0;
            }
            fineAdjust = 1 - fineAdjust;

            if (fineAdjustOn) scaleDrive(fineAdjust, driveValues);    //Apply Fine Adjust


            //Now maximize speed by applying a speed boost
            //The drive calculation sometimes doesn't set the peak drive to 1, this corrects that
            if (!fineAdjustOn & speedBoostOn){      //Fine Adjust mode takes precedent over speed boost
                maxValue = findMaxAbsValue(driveValues);  //See what the max drive value is set to
                if (maxValue < 1 & maxValue > 0) scaleDrive(1/maxValue, driveValues);  //If max value isn't 1, Scale the values up so max is 1
            }

            //Now actually assign the calculated drive values to the motors in motorList
            int i=0;
            for (DcMotor m: motorList){
                m.setPower(driveValues[i]);
                i++;
            }

            //GAMEPAD 2 INPUTS
            //  Y Button        --> arm angle positive direction
            //  A Button        --> arm angle negative direction
            //  Right Bumper    --> arm angle in dump position
            //  dpad UP         --> arm extension positive direction
            //  dpad DOWN       --> arm extension negative direction
            //  Right Trigger   --> collection motors positive
            //  Left Trigger    --> collection motor negative
            //  Left Stick      --> move latch up/down
            //  Right Stick     --> arm angle up/down
            //  Left Bumper     --> encoder override

            encoderSafetyOverride = true;
            /*
            if (gamepad2.left_bumper) {
                encoderSafetyOverride = true;
            } else {
                encoderSafetyOverride = false;
            }
            */
            //Manipulator controls

            armAngleDrive = gamepad2.right_stick_y;

            //Note that the armAngleMotor is setup so encoder values read negative
            //If the joystick is moved more than threshold
            if (armAngleDrive > motorThreshold){
                //if trying to extend, make sure that the position is within encoder safety limit
                if (armAngleMotor.getCurrentPosition() > ARM_ANGLE_LIMIT   //GREATER THAN because of negative orientation
                        | encoderSafetyOverride){
                    armAngleDrive = 1;
                }
            } else if (armAngleDrive < -motorThreshold){
                //if trying to retract, make sure that the position is above zero
                if (armAngleMotor.getCurrentPosition() < 0
                        | encoderSafetyOverride){
                    armAngleDrive = -1;
                }
            } else {
                armAngleDrive = 0;
            }

            if (!armAngleMotor.isBusy()) {
                armAngleMotor.setPower(armAngleDrive);
            }

            /*
            if(!armAngleMotor.isBusy()) {
                if (gamepad2.y) {
                    armAngleMotor.setPower(1);
                } else if (gamepad2.a) {
                    armAngleMotor.setPower(-1);
                } else {
                    armAngleMotor.setPower(0);
                }
            }
            */
            if(!armExtensionMotor.isBusy()) {
                if ((gamepad2.dpad_up & armExtensionMotor.getCurrentPosition() < ARM_EXTENSION_LIMIT)
                        | (gamepad2.dpad_up & encoderSafetyOverride)) {
                    armExtensionMotor.setPower(-1);
                } else if ((gamepad2.dpad_down & armExtensionMotor.getCurrentPosition() > 0)
                        | (gamepad2.dpad_down & encoderSafetyOverride)) {
                    armExtensionMotor.setPower(1);
                } else {
                    armExtensionMotor.setPower(0);
                }
            }


            //Get input for the collector motors
            rightTrigger = gamepad2.right_trigger;
            leftTrigger=gamepad2.left_trigger;

            if (rightTrigger > motorThreshold) {
                collectMotor.setPower(rightTrigger);
            } else if (leftTrigger>motorThreshold) {
                collectMotor.setPower(-leftTrigger);
            }else {
                collectMotor.setPower(0);
            }

            //Set to dump position
            /*
            if (gamepad2.right_bumper & !armAngleMotor.isBusy()){
                targetAngle = 0;
                targetEncoderCounts = (int) (degreesToEncoderCounts * targetAngle);
                armAngleMotor.setTargetPosition(targetEncoderCounts);
                armAngleMotor.setPower(1);
            }*/

            //Set Arm down to "collect" position
            if (gamepad2.a){
                armAngleMotor.setTargetPosition(ARM_ANGLE_COLLECT_POSITION);
                armExtensionMotor.setTargetPosition(ARM_EXTENSION_COLLECTION_POSITION);
                armAngleMotor.setPower(1);
                armExtensionMotor.setPower(1);
            }

            latchDrive = gamepad2.left_stick_y;

            if (latchDrive > motorThreshold){
                if(latchMotor.getCurrentPosition() > 0 | encoderSafetyOverride){
                    latchDrive = 1;
                }
            } else if (latchDrive < -motorThreshold){
                if(latchMotor.getCurrentPosition() < LATCH_LIMIT | encoderSafetyOverride){
                    latchDrive = -1;
                }
            } else {
                latchDrive = 0;
            }

            if (!latchMotor.isBusy()) {
                latchMotor.setPower(latchDrive);
            }
            //telemetry.addData("driveX", driveX);
            //telemetry.addData("driveY", driveY);
            //telemetry.addData("spin", spin);
            telemetry.addData("armAngleMotor Position", armAngleMotor.getCurrentPosition());
            telemetry.addData("armExtensionMotor Position", armExtensionMotor.getCurrentPosition());
            telemetry.addData("latchMotor Position", latchMotor.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
