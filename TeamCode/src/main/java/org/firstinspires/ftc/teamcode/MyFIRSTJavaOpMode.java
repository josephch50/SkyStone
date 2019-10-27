package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode {
//    private Gyroscope imu;
//    private DcMotor motorTest;
//    private DigitalChannel digitalTouch;
//    private DistanceSensor sensorColorRange;
//    private Servo servoTest;

    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorArmAngle;
    private DcMotor motorArmExtender;
    private Servo servoRH;
    private Servo servoGR;
    private Servo servoLH;
    private Servo servoGP;
    //private boolean hookIsDown = false;
    //private boolean leftTriggerIsPressed = false;
    double interval = 0.05;
    private double currentGripperRotatePosition;

    @Override
    public void runOpMode() {
//        imu = hardwareMap.get(Gyroscope.class, "imu");
//        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
//        servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Define Motors & Servos
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorArmAngle = hardwareMap.get(DcMotor.class, "motorAA");
        motorArmExtender = hardwareMap.get(DcMotor.class, "motorAE");
        servoRH = hardwareMap.get(Servo.class, "servoRH");
        servoGR = hardwareMap.get(Servo.class, "servoGR");
        servoLH = hardwareMap.get(Servo.class, "servoLH");
        servoGP = hardwareMap.get(Servo.class, "servoGP");

        // Set Motor Power
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorArmAngle.setPower(0);
        motorArmExtender.setPower(0);

        // Set Motor Mode
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Right Motors to reverse values
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        servoLH.setDirection(Servo.Direction.REVERSE);

        // Set Motor zeroPowerBehavior
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servo Position
        servoRH.setPosition(0.5);
        servoGR.setPosition(0.85);
        servoLH.setPosition(0.5);
        servoGP.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            wheelCtrl();
        }
    }

    private void wheelCtrl() {

        //GAMEPAD 2
        double gamepad2LeftJoystickX = -gamepad2.left_stick_x; //
        double gamepad2LeftJoystickY = -gamepad2.left_stick_y; //

        double gamepad2RightJoystickX = -gamepad2.right_stick_x; //
        double gamepad2RightJoystickY = -gamepad2.right_stick_y; //

        //GAMEPAD 2

        //GAMEPAD 1
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double leftJoystickX = -gamepad1.left_stick_x; // Used to move robot left/right
        double leftJoystickY = -gamepad1.left_stick_y; // Used to move robot forward/backward

        double rightJoystickX = -gamepad1.right_stick_x; // Used to turn robot left(Counter-Clockwise)/right(Clockwise)
        double rightJoystickY = -gamepad1.right_stick_y; // Unused
        //GAMEPAD 1


//        if (gamepad2.left_trigger > 0) {
//            leftTriggerIsPressed = true;
//        } else if (gamepad2.left_trigger == 0) {
//            leftTriggerIsPressed = false;
//        }

//        if (gamepad1.y) {
//            if (hookIsDown) {
//                servoRH.setPosition(0.5);
//                servoLH.setPosition(0.5);
//                hookIsDown = false;
//            } else {
//                servoRH.setPosition(1);
//                servoLH.setPosition(1);
//                hookIsDown = true;
//            }
//        }

        if (gamepad1.y) {
            servoRH.setPosition(0.5);
            servoLH.setPosition(0.5);
        } else if (gamepad1.x) {
            servoRH.setPosition(1);
            servoLH.setPosition(1);
        }


        //SENDING DATA TO PHONE WITH TELEMETRY
        telemetry.addData("Gamepad 2 Dpad Down Is Pressed", gamepad2.dpad_down);
        telemetry.addData("Left Joystick (X,Y)", "(" + leftJoystickX + "," + leftJoystickY + ")");
        telemetry.addData("Right Joystick (X,Y)", "(" + rightJoystickX + "," + rightJoystickY + ")");
        //SENDING DATA TO PHONE WITH TELEMETRY


        // Setting the motors to a negative value will cause the robot to go forwards
        if (leftJoystickY < 0) { // Left Joystick Down (Backwards)
            telemetry.addData("Direction", "Down"); //GOING BACKWARDS, JOYSTICK DOWN
            motorFL.setPower(leftJoystickY); // -1
            motorFR.setPower(leftJoystickY); // -1
            motorBL.setPower(leftJoystickY); // -1
            motorBR.setPower(leftJoystickY); // -1
        } else if (leftJoystickY > 0) { // Left Joystick Up (Forwards)
            telemetry.addData("Direction", "Up"); //GOING FORWARDS WITH STICK UP
            motorFL.setPower(leftJoystickY); // 1
            motorFR.setPower(leftJoystickY); // 1
            motorBL.setPower(leftJoystickY); // 1
            motorBR.setPower(leftJoystickY); // 1
        } else if (leftJoystickX > 0) { // Left Joystick Left (Left)
            telemetry.addData("Direction", "Left"); //GOING LEFT WITH STICK SIDEWAYS LEFT
            motorFL.setPower(-leftJoystickX);  //  1
            motorFR.setPower(leftJoystickX);   // -1
            motorBL.setPower(leftJoystickX);   // -1
            motorBR.setPower(-leftJoystickX);  //  1
        } else if (leftJoystickX < 0) { // Left Joystick Right (Right)
            telemetry.addData("Direction", "Right"); //GOING RIGHT WITH STICK SIDEWAYS RIGHT
            motorFL.setPower(-leftJoystickX);  //  1
            motorFR.setPower(leftJoystickX);   // -1
            motorBL.setPower(leftJoystickX);   // -1
            motorBR.setPower(-leftJoystickX);  //  1
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }


        // TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - //
        if (rightJoystickX > 0) { // Right Joystick Left (Counter Clockwise)
            telemetry.addData("Spin", "Counter Clockwise"); //TURNING COUNTER CLOCKWISE WITH STICK LEFT
            motorFL.setPower(-rightJoystickX); //  1
            motorFR.setPower(rightJoystickX);  // -1
            motorBL.setPower(-rightJoystickX); //  1
            motorBR.setPower(rightJoystickX);  // -1
        } else if (rightJoystickX < 0) { // Right Joystick Right (Clockwise)
            telemetry.addData("Spin", "Clockwise"); //TURNING CLOCKWISE WITH STICK RIGHT
            motorFL.setPower(-rightJoystickX); // -1
            motorFR.setPower(rightJoystickX);  //  1
            motorBL.setPower(-rightJoystickX); // -1
            motorBR.setPower(rightJoystickX);  //  1
        }
        // TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - TURNING - //


        //ARM ANGLE CHANGED WITH RIGHT JOYSTICK
        if (gamepad2RightJoystickY < 0) {
            motorArmAngle.setPower(-gamepad2RightJoystickY);
        } else if (gamepad2RightJoystickY > 0) {
            motorArmAngle.setPower(-gamepad2RightJoystickY);
        } else {
            motorArmAngle.setPower(0);
        }
        //ARM ANGLE CHANGED WITH RIGHT JOYSTICK


        //ARM EXTENDER CHANGED WITH DPAD
        if (gamepad2.dpad_up) { //joystick down
            motorArmExtender.setPower(0.75); //negative, e.x. -1
        } else if (gamepad2.dpad_down) { //joystick down
            motorArmExtender.setPower(-0.75); //negative, e.x. -1
        } else {
            motorArmExtender.setPower(0);
        }
        //ARM EXTENDER CHANGED WITH DPAD


        //Gripper Wrist Position Set With X and Y position
        if (gamepad2LeftJoystickX > 0) { //joystick down
            currentGripperRotatePosition = 0.85;
        } else if (gamepad2LeftJoystickX < 0) { //joystick down
            currentGripperRotatePosition = 0.3;
        }
        //Gripper Wrist Position Set With X and Y position


        // Click 'A' pincher goes up, Click 'X' and pincher pinches
        if (gamepad2.a) {
            servoGP.setPosition(0);
        } else if (gamepad2.x) {
            servoGP.setPosition(1);
        }
        // Click 'A' pincher goes up, Click 'X' and pincher pinches


        //Click 'B' and Gripper Wrist goes down about 0.7 further for more precision for being parallel to the ground/block.
        if (gamepad2.b) {
            currentGripperRotatePosition = 0.17;
        }

        if (gamepad2.y && gamepad2LeftJoystickY < 0) { //Change Gripper Wrist angle by 0.1 per refresh.
            currentGripperRotatePosition += interval;
            if (currentGripperRotatePosition >= 0.85) {
                currentGripperRotatePosition = 0.85;
            }
        }
        // Joystick down, Gripper Rotate Down
        if (gamepad2.y && gamepad2LeftJoystickY > 0) { //Change Gripper Wrist angle by 0.1 per refresh.
            currentGripperRotatePosition -= interval;
            if (currentGripperRotatePosition <= 0.15) {
                currentGripperRotatePosition = 0.15;
            }
        }

        servoGR.setPosition(currentGripperRotatePosition);

        //Gripper wrist Control



//        if (gamepad2RightJoystickX < 0.05 && gamepad2RightJoystickX > -0.05 && rightJoystickX > -0.05 && rightJoystickY < 0.05
//                && rightJoystickY > -0.05 && leftJoystickX > -0.05 && leftJoystickX < 0.05){
//            motorFL.setPower(0);
//            motorFR.setPower(0);
//            motorBL.setPower(0);
//            motorBR.setPower(0);
//            motorArmAngle.setPower(0);
//        }

        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorArmAngle.setPower(0);
        }
    }
    }







