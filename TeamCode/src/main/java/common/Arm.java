/*
 * This file contains the code for the arm.
 */

package common;

import static java.lang.Math.cos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

public class Arm {

    public enum ARM_POSITION { HOME, LOW, HIGH }
    public enum ARM_STATE { NONE, MOVE_HOME, MOVE_LOW, MOVE_HIGH, DROP_SAMPLE, ARM_UP }

    static final double ELBOW_SPEED = 1;
    static final double ARM_SPEED = 1;

    private final double COUNTS_PER_MOTOR_REV = 28;              // Gobilda 5203 Yellow Jacket
    private final double DRIVE_GEAR_REDUCTION = 26.9;              // Gearing
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches

    private final double encoderDegree = 42.22; //encoder cts per degree
    private final double encoderInch = -1129 / 7.311;

    public double length = 0;

    // Position for all the pixel arm servos and motor encoders
    public static final int    ELBOW_DOWN      = 0;
    public static final int    ELBOW_UP_LOW    = -2245; //ToDo edit elbow value
    public static final int    ELBOW_UP_HIGH   = -2990; //ToDo edit elbow value

    public static final int    ARM_IN          = 0;
    public static final int    ARM_OUT_LOW     = -3380; //ToDo edit arm value
    public static final int    ARM_OUT_HIGH    = -4900; //ToDo edit arm value

    public static final double WRIST_HOME         = 0;
    public static final double WRIST_DROP_LOW     = 0.44;
    public static final double WRIST_DROP_HIGH    = 0.445;
    public static final double WRIST_SPECIMEN     = 0;

    private DcMotor elbow = null;
    private DcMotor leftArm   = null;
    private DcMotor rightArm = null;
    private CRServo   wrist = null;
    private CRServo intake = null;

    public ARM_STATE state  = ARM_STATE.NONE;
    private final ElapsedTime stateTime = new ElapsedTime();

    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    public enum intakeStates { OFF, FORWARD, REVERSE }
    public intakeStates iState = intakeStates.OFF;

    private boolean armActive = false;

    public LinearOpMode opMode;

    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init() {
        try {
            leftArm = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_ARM);
            rightArm = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_ARM);
            elbow = opMode.hardwareMap.get(DcMotorEx.class, Config.ELBOW);
            wrist = opMode.hardwareMap.get(CRServo.class, Config.WRIST);
            intake = opMode.hardwareMap.get(CRServo.class, Config.INTAKE);

            leftArm.setDirection(DcMotor.Direction.REVERSE);
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightArm.setDirection(DcMotor.Direction.FORWARD);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            elbow.setDirection(DcMotor.Direction.REVERSE);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0);

            //wrist.setPosition(WRIST_HOME);
            armMove(ARM_IN);

        } catch (Exception e) {
            Logger.error(e, "Pixel arm hardware not found");
        }
    }

    /**
     * Move the arm elbow to the specified position
     *
     * @param newPosition position to move to
     */
    public void elbowMove(int newPosition) {
        opMode.telemetry.addData("elbow", "elbowMove %d", newPosition);

        int from = elbow.getCurrentPosition();
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setTargetPosition(newPosition);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(Math.abs(ELBOW_SPEED));

        /*
        while (elbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        elbow.setPower(0);
        */

        Logger.message("move elbow from %d to %d", from, newPosition);
    }

    /**
     * Extend or retract the pixel arm to the specified position. The home position is zero.
     *
     * @param newPosition position to move to
     */
    public void armMove(int newPosition) {
        Logger.message("armMove %d", newPosition);
        int from = leftArm.getCurrentPosition();
        leftArm.setTargetPosition(newPosition);
        rightArm.setTargetPosition(newPosition);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(Math.abs(ARM_SPEED));
        rightArm.setPower(Math.abs(ARM_SPEED));
        Logger.message("move arm from %d to %d (%d)", from, newPosition, leftArm.getCurrentPosition() );

        /*
        while (pixelArm.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }

        if (newPosition == 0) {
            // only stop the motor when the arm is lowered
            pixelArm.setPower(0);
        }
        */
    }

    /*public void wristMove(double position) {
        wrist.setPosition(position);
    }*/

    public void intakeSet(intakeStates setState) {
        if (setState == intakeStates.FORWARD) {
            intake.setPower(1);
        } else if (setState == intakeStates.REVERSE) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
        iState = setState;
    }

    public void enablelArm (boolean enable) {
        armActive = enable;
    }


    public void run () {
        length = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(elbow.getCurrentPosition() / encoderDegree));
        opMode.telemetry.addData("length", length);
        //Logger.message("run");

        if (armActive) {
            //Logger.message("armActive");
            control();
        }

        if (state == ARM_STATE.NONE) {
            //Logger.message("none");
            return;
        }

        if (state == ARM_STATE.MOVE_LOW) {
            //Logger.message("low");
            if (stateTime.milliseconds() < 500)
                return;
            armMove((ARM_OUT_LOW));
            //wristMove(WRIST_DROP_LOW);
            state = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to low position");


        } else if (state == ARM_STATE.MOVE_HIGH) {
            //Logger.message("high");
            if (stateTime.milliseconds() < 500)
                return;
            armMove((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
            state = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to high position");

        } else if (state == ARM_STATE.MOVE_HOME) {
            Logger.message("home");
            if (stateTime.milliseconds() < 1000)
                return;
            elbowMove(ELBOW_DOWN);
            Logger.message("pixel elbow to home position");

            if (leftArm.isBusy())
                return;
            leftArm.setPower(0);
            rightArm.setPower(0);
            state = ARM_STATE.NONE;
            Logger.message("pixel arm power off");

        } else if (state == ARM_STATE.ARM_UP) {
            Logger.message("up");
            if (stateTime.milliseconds() < 10000)
                return;
            leftArm.setPower(0);
            rightArm.setPower(0);
            state = ARM_STATE.NONE;
        }
    }

    public void dropSample () {
        state = ARM_STATE.DROP_SAMPLE;
        stateTime.reset();
    }

    public void positionArmAsyn(ARM_POSITION position) {

        stateTime.reset();
        if (position == ARM_POSITION.LOW) {
            elbowMove(ELBOW_UP_LOW);
            Logger.message("pixel elbow to low position");
            state = ARM_STATE.MOVE_LOW;
        } else if (position == ARM_POSITION.HIGH) {
            elbowMove(ELBOW_UP_HIGH);
            state = ARM_STATE.MOVE_HIGH;
            Logger.message("pixel elbow to high position");
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMove(ARM_IN);
            state = ARM_STATE.MOVE_HOME;
            Logger.message("pixel wrist and arm to home position");
        }
    }

    public void positionArm(ARM_POSITION position) {

        if (position == ARM_POSITION.LOW) {
            elbowMove(ELBOW_UP_LOW);
            opMode.sleep(500);
            armMove((ARM_OUT_LOW));
            //wristMove(WRIST_DROP_LOW);
        } else if (position == ARM_POSITION.HIGH) {
            elbowMove(ELBOW_UP_HIGH);
            armMove((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMove(ARM_IN);
            elbowMove(ELBOW_DOWN);
            while (leftArm.isBusy()) {
                if (!opMode.opModeIsActive()) {
                    break;
                }
            }
            leftArm.setPower(0);
            rightArm.setPower(0);
        }
    }

    public boolean positionCommand () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.a || gamepad.b || gamepad.x);
    }

    public boolean dropCommand () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.right_trigger > 0);

    }

    public void displayControls(){
        opMode.telemetry.addLine("Pixel Arm Controls (Gamepad 2)\n" +
                        "  a - position arm at low position\n" +
                        "  b - position arm at mid position\n" +
                        "  x - position arm at high position\n" +
                        "  y - position arm at pickup position\n" +
                        "  right triggers - drop pixels\n" +
                        elbow.getCurrentPosition() + "\n"
                //"  left stick - move elbow (u/d)  arm (l/r)\n" +
                //"  right stick - manual rotate the hands\n"
        );
    }

    /**
     * Manually control the pixel arm
     */
    public boolean control() {
        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        run();   // ToDo remove when subclass from thread

        if (gamepad.back) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad.a) {
            // Move the arm to the lower drop position
            if (! aPressed) {
                positionArmAsyn(ARM_POSITION.LOW);
                aPressed = true;
            }
        } else {
            aPressed = false;
        }

        /*if (gamepad.x) {
            // Move the arm to the middle drop position
            if (!xPressed) {
                positionArmAsyn(ARM_POSITION.MID);
                xPressed = true;
            }
        } else {
            xPressed = false;
        }*/

        if (gamepad.b) {
            // Move the arm to the higher drop position
            if (!bPressed) {
                positionArmAsyn(ARM_POSITION.HIGH);
                bPressed = true;
            }
        } else {
            bPressed = false;
        }

        if (gamepad.y) {
            // Move the arm to the home position
            if (!yPressed) {
                positionArmAsyn(ARM_POSITION.HOME);
                intake.setPower(0);
                iState = intakeStates.OFF;
                yPressed = true;
            }
        } else {
            yPressed = false;
        }

        if (gamepad.left_stick_y != 0) {
            // manually move the elbow
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad.left_stick_y > 0)
                elbow.setPower(ELBOW_SPEED);
            else if (gamepad.left_stick_y < 0)
                elbow.setPower(-ELBOW_SPEED);
            while (true) {
                length = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(elbow.getCurrentPosition() / encoderDegree));
                if (gamepad.left_stick_y == 0 || length >= 30) {
                    while (length >= 30) {
                        elbow.setPower(-ELBOW_SPEED);
                        length = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(elbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            elbow.setPower(0);
            Logger.message( "elbow position %7d", elbow.getCurrentPosition());

        } else if (gamepad.left_stick_x != 0) {
            // manually extend or retract the pixel arm
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad.left_stick_x > 0) {
                leftArm.setPower(ARM_SPEED);
                rightArm.setPower(ARM_SPEED);
            } else if (gamepad.left_stick_x < 0) {
                leftArm.setPower(-ARM_SPEED);
                rightArm.setPower(-ARM_SPEED);
            }
            while (true) {
                length = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(elbow.getCurrentPosition() / encoderDegree));
                if (gamepad.left_stick_x == 0 || length >= 30){
                        while (length>=30) {
                            leftArm.setPower(ARM_SPEED);
                            rightArm.setPower(ARM_SPEED);
                            length = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(elbow.getCurrentPosition() / encoderDegree));
                        }
                    break;
                }
            }
            int position = leftArm.getCurrentPosition();
            leftArm.setTargetPosition(position);
            rightArm.setTargetPosition(position);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Logger.message( "arm position %7d", position);

        } else if (gamepad.right_stick_y != 0) {
            // manually rotate the bucket
            /*while (gamepad.right_stick_y != 0) {
                double position = wrist.getPosition();
                if (Double.isNaN(position))
                    position = WRIST_HOME;
                else if (gamepad.right_stick_y > 0)
                    position += 0.0125;
                else if (gamepad.right_stick_y < 0)
                    position -= 0.0125;
                wrist.setPosition(position);
                opMode.sleep(100);
            }
            Logger.message("wrist position %f", wrist.getPosition());*/
            wrist.setPower(gamepad.right_stick_y);
            Logger.message("wrist position %f", wrist.getPower());

        } else {
            wrist.setPower(gamepad.right_stick_y);
            handled = false;
        }

        if(gamepad.right_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.REVERSE) {
                intake.setPower(1);
                iState = intakeStates.FORWARD;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        } else if(gamepad.left_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.FORWARD) {
                intake.setPower(-1);
                iState = intakeStates.REVERSE;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        }

        return handled;
    }
}