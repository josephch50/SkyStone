package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        motorFL.setPower(-0);
        motorFR.setPower(0);
        motorBL.setPower(-0);
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

        // Set Motor zeroPowerBehavior
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Servo Position
        servoRH.setPosition(0);
        servoGR.setPosition(0);
        servoLH.setPosition(0);
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

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double leftJoystickX = -gamepad1.left_stick_x; // Used to move robot left/right
        double leftJoystickY = -gamepad1.left_stick_y; // Used to move robot forward/backward

        double rightJoystickX = -gamepad1.right_stick_x; // Used to turn robot left(Counter-Clockwise)/right(Clockwise)
        double rightJoystickY = -gamepad1.right_stick_y; // Unused

        if (leftJoystickY >= -1 && leftJoystickY < 0) { // Left Joystick Down
            motorFL.setPower(-leftJoystickY);
            motorFR.setPower(leftJoystickY);
            motorBL.setPower(-leftJoystickY);
            motorBR.setPower(leftJoystickY);
        } else if (leftJoystickY <= 1 && leftJoystickY > 0) { // Left Joystick Up
            motorFL.setPower(-leftJoystickY);
            motorFR.setPower(leftJoystickY);
            motorBL.setPower(-leftJoystickY);
            motorBR.setPower(leftJoystickY);
        } else if (leftJoystickX <= 1 && leftJoystickX > 0){ // Left Joystick Left
            //robot going sideways toward the left
            //left joystick is going towards the left
            motorFL.setPower(leftJoystickX);
            motorFR.setPower(leftJoystickX);
            motorBL.setPower(-leftJoystickX);
            motorBR.setPower(-leftJoystickX);
        } else if (leftJoystickX >= -1 && leftJoystickX > 0){ // Left Joystick Right
            //robot going sideways toward the right
            //left joystick is going towards the right
            motorFL.setPower(-leftJoystickX);
            motorFR.setPower(-leftJoystickX);
            motorBL.setPower(leftJoystickX);
            motorBR.setPower(leftJoystickX);
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }

        //turn robot
        if (rightJoystickX <= 1 && rightJoystickX > 0){
            motorFL.setPower(-rightJoystickX);
            motorFR.setPower(-rightJoystickX);
            motorBL.setPower(-rightJoystickX);
            motorBR.setPower(-rightJoystickX);
        } else if (rightJoystickX >= -1 && rightJoystickX < 0){
            motorFL.setPower(rightJoystickX);
            motorFR.setPower(rightJoystickX);
            motorBL.setPower(rightJoystickX);
            motorBR.setPower(rightJoystickX);
        }
    }

    private void armCtrl() {

    }
}




