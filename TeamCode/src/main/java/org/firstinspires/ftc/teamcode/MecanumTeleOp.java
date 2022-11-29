package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
       // Integers referenced later on
        int LIFT_POS_GRAB = 0;
        int LIFT_POS_HIGH = 3000;
        int LIFT_POS_MEDIUM = 2150;
        int LIFT_POS_LOW = 1300;
        int step = 100;

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");
        DcMotor lift = hardwareMap.dcMotor.get("l");
        DcMotor angle = hardwareMap.dcMotor.get("a");
        CRServo grip1 = hardwareMap.get(CRServo.class, "g1");
        CRServo grip2 = hardwareMap.get(CRServo.class, "g2");
        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        DigitalChannel digitalTouch;
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.setAutoClear(false);
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift.getCurrentPosition());
        liftPosition.setValue(lift.getCurrentPosition());
        telemetry.update();
        float driveSpeed = 0.7f; //sets drive motor speeds (between 0 and 1)

        waitForStart();

        if (isStopRequested()) return;

        // Setting Gamepad Values to variables
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double l = gamepad2.left_stick_y;
            double a = gamepad2.right_stick_y;

        // Math for Mecanum drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double liftPower = l;
            double anglePower = a;
            double armSpeedUp = 1;
            double armSpeedDown = -0.4;

            // Drive Control
            frontLeft.setPower(frontLeftPower * driveSpeed);
            backLeft.setPower(backLeftPower * driveSpeed);
            frontRight.setPower(frontRightPower * driveSpeed);
            backRight.setPower(backRightPower * driveSpeed);
            angle.setPower(anglePower);

            // Gripper Control
            if (gamepad2.right_trigger > 0.2 && digitalTouch.getState() == false) { // intake
                grip1.setPower(gamepad2.right_trigger / 2);
                grip2.setPower(-gamepad2.right_trigger / 2);
                LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (gamepad2.left_trigger > 0.2) { // release
                grip1.setPower(-gamepad2.left_trigger / 2);
                grip2.setPower(gamepad2.left_trigger / 2);
                LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else { // no input
                grip1.setPower(0);
                grip2.setPower(0);
                LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

                // Automated Levels
                if (gamepad2.a) { // high goal
                    lift.setPower(armSpeedUp);
                    lift.setTargetPosition(LIFT_POS_HIGH);
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.b) { // cone pickup position
                    //insert gripper automation code
                    lift.setPower(armSpeedDown);
                    lift.setTargetPosition(LIFT_POS_GRAB);
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.x) { // medium goal
                    lift.setPower(armSpeedUp);
                    lift.setTargetPosition(LIFT_POS_MEDIUM);
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.y) { // low goal
                    lift.setPower(armSpeedUp);
                    lift.setTargetPosition(LIFT_POS_LOW);
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.dpad_up) { // manual mode
                    lift.setPower(armSpeedUp);
                    lift.setTargetPosition((lift.getCurrentPosition() + step));
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } else if (gamepad2.dpad_down){
                    lift.setPower(armSpeedDown);
                    lift.setTargetPosition((lift.getCurrentPosition() - step));
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                } if (lift.isBusy()) {
                    liftPosition.setValue(lift.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
    }
}

