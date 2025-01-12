/*
 * This file contains the code for the arm.
 */

package common;

import static java.lang.Math.cos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmOneThread {

    //gamepad1 drives left, gamepad2 drives right

    private double msCt1;
    private double msCt2;

    public enum ARM_POSITION { HOME, HIGH, SAMPLE, RUNG }
    public enum ARM_STATE { NONE, MOVE_HOME, MOVE_HIGH, MOVE_SAMPLE, MOVE_RUNG, ARM_UP, DROP_SAMPLE }

    static final double ELBOW_SPEED = 1;
    static final double ARM_SPEED = 1;

    private final double COUNTS_PER_MOTOR_REV = 28;              // Gobilda 5203 Yellow Jacket
    private final double DRIVE_GEAR_REDUCTION = 26.9;              // Gearing
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches

    private final double encoderDegree = 31.11; //encoder cts per degree 42.22
    private final double encoderInch = -1129 / 7.311;

    public double lengthLeft = 0;
    public double lengthRight = 0;

    // Position for all the pixel arm servos and motor encoders
    public static final int    ELBOW_DOWN      = 0;
    public static final int    ELBOW_SAMPLE    = -464; //ToDo edit elbow value
    public static final int    ELBOW_RUNG      = -2182;
    public static final int    ELBOW_UP_HIGH   = -2203; //ToDo edit elbow value

    public static final int    ARM_IN          = 0;
    public static final int    ARM_SAMPLE     = -1434; //ToDo edit arm value
    public static final int    ARM_RUNG       = -1621;
    public static final int    ARM_OUT_HIGH    = -4900; //ToDo edit arm value

    public static final double WRIST_HOME         = 0;
    public static final double WRIST_RUNG         = 0;
    public static final double WRIST_DROP_HIGH    = 0.445;
    public static final double WRIST_SPECIMEN     = 0;

    private DcMotor leftElbow = null;
    private DcMotor rightElbow = null;
    private DcMotor leftArm   = null;
    private DcMotor rightArm = null;
    private CRServo wristRight = null;
    public Servo vel = null;
    public Servo accel = null;
    public Servo specimenLeft = null;
    public Servo specimenRight = null;
    public ARM_STATE stateLeft  = ARM_STATE.NONE;
    public ARM_STATE stateRight  = ARM_STATE.NONE;
    private final ElapsedTime stateTime = new ElapsedTime();

    private boolean aPressed1 = false;
    private boolean bPressed1 = false;
    private boolean xPressed1 = false;
    private boolean yPressed1 = false;

    private boolean aPressed2 = false;
    private boolean bPressed2 = false;
    private boolean xPressed2 = false;
    private boolean yPressed2 = false;

    public enum intakeStates { OFF, FORWARD, REVERSE }
    public intakeStates wStateRight = intakeStates.OFF;

    private boolean armActiveLeft = false;
    private boolean armActiveRight = false;

    public LinearOpMode opMode;

    public ArmOneThread(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init() {
        try {
            leftArm = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_ARM);
            rightArm = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_ARM);
            leftElbow = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_ELBOW);
            rightElbow = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_ELBOW);
            vel = opMode.hardwareMap.get(Servo.class, Config.VEL);
            accel = opMode.hardwareMap.get(Servo.class, Config.ACCEL);
            //wristRight = opMode.hardwareMap.get(CRServo.class, Config.WRIST_RIGHT);
            specimenLeft = opMode.hardwareMap.get(Servo.class, Config.SPECIMEN_LEFT);
            specimenRight = opMode.hardwareMap.get(Servo.class, Config.SPECIMEN_RIGHT);

            leftArm.setDirection(DcMotor.Direction.REVERSE);
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightArm.setDirection(DcMotor.Direction.FORWARD);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftElbow.setDirection(DcMotor.Direction.REVERSE);
            leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightElbow.setDirection(DcMotor.Direction.REVERSE);
            rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //wrist.setPosition(WRIST_HOME);
            armMoveLeft(ARM_IN);
            armMoveLeft(ARM_IN);

        } catch (Exception e) {
            Logger.error(e, "Pixel arm hardware not found");
        }
    }

    /**
     * Move the arm elbow to the specified position
     *
     * @param newPosition position to move to
     */
    public void elbowMoveLeft(int newPosition) {
        opMode.telemetry.addData("elbowLeft", "elbowMove %d", newPosition);

        int from = leftElbow.getCurrentPosition();
        leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElbow.setTargetPosition(newPosition);
        leftElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElbow.setPower(Math.abs(ELBOW_SPEED));

        /*
        while (elbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        elbow.setPower(0);
        */

        Logger.message("move elbowLeft from %d to %d", from, newPosition);
    }

    public void elbowMoveRight(int newPosition) {
        opMode.telemetry.addData("elbowRight", "elbowMove %d", newPosition);

        int from = rightElbow.getCurrentPosition();
        rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElbow.setTargetPosition(newPosition);
        rightElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElbow.setPower(Math.abs(ELBOW_SPEED));

        /*
        while (elbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        elbow.setPower(0);
        */

        Logger.message("move elbowRight from %d to %d", from, newPosition);
    }

    public void elbowMove(int newPosition) {
        opMode.telemetry.addData("elbow", "elbowMove %d", newPosition);

        int fromL = leftElbow.getCurrentPosition();
        int fromR = rightElbow.getCurrentPosition();
        leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElbow.setTargetPosition(newPosition);
        rightElbow.setTargetPosition(newPosition);
        leftElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElbow.setPower(Math.abs(ELBOW_SPEED));
        rightElbow.setPower(Math.abs(ELBOW_SPEED));

        /*
        while (elbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        elbow.setPower(0);
        */

        Logger.message("move elbowRight from %d & d% to %d (%d) & (%d)", fromL, fromR, newPosition, leftElbow.getCurrentPosition(), rightArm.getCurrentPosition());
    }

    /**
     * Extend or retract the pixel arm to the specified position. The home position is zero.
     *
     * @param newPosition position to move to
     */
    public void armMoveLeft(int newPosition) {
        Logger.message("armMoveLeft %d", newPosition);
        int from = leftArm.getCurrentPosition();
        leftArm.setTargetPosition(newPosition);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(Math.abs(ARM_SPEED));
        Logger.message("move armLeft from %d to %d (%d)", from, newPosition, leftArm.getCurrentPosition());

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

    public void armMoveRight(int newPosition) {
        Logger.message("armMoveRight %d", newPosition);
        int from = rightArm.getCurrentPosition();
        rightArm.setTargetPosition(newPosition);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(Math.abs(ARM_SPEED));
        Logger.message("move arm from %d to %d (%d)", from, newPosition, rightArm.getCurrentPosition());

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

    public void armMove(int newPosition) {
        Logger.message("armMove %d", newPosition);
        int fromL = leftArm.getCurrentPosition();
        int fromR = rightArm.getCurrentPosition();
        leftArm.setTargetPosition(newPosition);
        rightArm.setTargetPosition(newPosition);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(Math.abs(ARM_SPEED));
        rightArm.setPower(Math.abs(ARM_SPEED));
        Logger.message("move arm from %d & %d to %d (%d) & (%d)", fromL, fromR, newPosition, leftArm.getCurrentPosition(), rightArm.getCurrentPosition());

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

    public void wristMoveRight(intakeStates setState) {
        if (setState == intakeStates.FORWARD) {
            wristRight.setPower(1);
        } else if (setState == intakeStates.REVERSE) {
            wristRight.setPower(-1);
        } else {
            wristRight.setPower(0);
        }
        wStateRight = setState;
    }

    /*public void intakeSet(intakeStates setState) {
        if (setState == intakeStates.FORWARD) {
            intake.setPower(1);
        } else if (setState == intakeStates.REVERSE) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
        iState = setState;
    }*/

    public void setSpecimenLeft(boolean enable) {
        if (enable) {
            specimenLeft.setPosition(0);
        } else {
            specimenLeft.setPosition(1);
        }
    }

    public void setSpecimenRight(boolean enable) {
        if (enable) {
            specimenRight.setPosition(0);
        } else {
            specimenRight.setPosition(1);
        }
    }

    public void setSpecimen(boolean enable) {
        if (enable) {
            specimenRight.setPosition(0);
            specimenLeft.setPosition(0);
        } else {
            specimenRight.setPosition(1);
            specimenLeft.setPosition(1);
        }
    }

    public void enablelArmLeft (boolean enable) {
        armActiveLeft = enable;
    }
    public void enablelArmRight (boolean enable) {
        armActiveRight = enable;
    }


    public void run () {
        lengthLeft = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
        lengthRight = (rightArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
        opMode.telemetry.addData("lengthLeft", lengthLeft);
        opMode.telemetry.addData("lengthRight", lengthRight);
        //Logger.message("run");

        if (armActiveLeft) {
            //Logger.message("armActive");
            controlLeft();
        }
        if (armActiveRight) {
            //Logger.message("armActive");
            controlRight();
        }

        if (stateLeft == ARM_STATE.NONE) {
            //Logger.message("none");
            return;
        }

        if (stateRight == ARM_STATE.NONE) {
            //Logger.message("none");
            return;
        }

        if (stateLeft == ARM_STATE.MOVE_SAMPLE) {
            //Logger.message("low");
            if (stateTime.milliseconds() < 500)
                return;
            armMoveLeft((ARM_SAMPLE));
            //wristMove(WRIST_DROP_LOW);
            stateLeft = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to low position");


        } else if (stateLeft == ARM_STATE.MOVE_HIGH) {
            //Logger.message("high");
            if (stateTime.milliseconds() < 500)
                return;
            armMoveLeft((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
            stateLeft = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to high position");

        } else if (stateLeft == ARM_STATE.MOVE_HOME) {
            Logger.message("home");
            if (stateTime.milliseconds() < 1000)
                return;
            elbowMoveLeft(ELBOW_DOWN);
            Logger.message("pixel elbow to home position");

            if (leftArm.isBusy())
                return;
            leftArm.setPower(0);
            rightArm.setPower(0);
            stateLeft = ARM_STATE.NONE;
            Logger.message("pixel arm power off");

        } else if (stateLeft == ARM_STATE.ARM_UP) {
            Logger.message("up");
            if (stateTime.milliseconds() < 10000)
                return;
            leftArm.setPower(0);
            rightArm.setPower(0);
            stateLeft = ARM_STATE.NONE;
        }

        if (stateRight == ARM_STATE.MOVE_SAMPLE) {
            //Logger.message("low");
            if (stateTime.milliseconds() < 500)
                return;
            armMoveRight((ARM_SAMPLE));
            //wristMove(WRIST_DROP_LOW);
            stateRight = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to low position");


        } else if (stateRight == ARM_STATE.MOVE_HIGH) {
            //Logger.message("high");
            if (stateTime.milliseconds() < 500)
                return;
            armMoveRight((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
            stateRight = ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to high position");

        } else if (stateRight == ARM_STATE.MOVE_HOME) {
            Logger.message("home");
            if (stateTime.milliseconds() < 1000)
                return;
            elbowMoveRight(ELBOW_DOWN);
            Logger.message("pixel elbow to home position");

            if (rightArm.isBusy())
                return;
            rightArm.setPower(0);
            rightArm.setPower(0);
            stateRight = ARM_STATE.NONE;
            Logger.message("pixel arm power off");

        } else if (stateRight == ARM_STATE.ARM_UP) {
            Logger.message("up");
            if (stateTime.milliseconds() < 10000)
                return;
            rightArm.setPower(0);
            rightArm.setPower(0);
            stateRight = ARM_STATE.NONE;
        }
    }

    public void dropSampleLeft () {
        stateLeft = ARM_STATE.DROP_SAMPLE;
        stateTime.reset();
    }

    public void dropSampleRight () {
        stateRight = ARM_STATE.DROP_SAMPLE;
        stateTime.reset();
    }

    public void positionArmAsynLeft(ARM_POSITION position) {

        stateTime.reset();
        if (position == ARM_POSITION.SAMPLE) {
            elbowMoveLeft(ELBOW_SAMPLE);
            Logger.message("elbow to sample position");
            stateLeft = ARM_STATE.MOVE_SAMPLE;
        } else if (position == ARM_POSITION.RUNG) {
            elbowMoveLeft(ELBOW_RUNG);
            stateLeft = ARM_STATE.MOVE_RUNG;
            Logger.message("elbow to rung position");
        } else if (position == ARM_POSITION.HIGH) {
            elbowMoveLeft(ELBOW_UP_HIGH);
            stateLeft = ARM_STATE.MOVE_HIGH;
            Logger.message("elbow to high position");
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMoveLeft(ARM_IN);
            stateLeft = ARM_STATE.MOVE_HOME;
            Logger.message("wrist and arm to home position");
        }
    }

    public void positionArmAsynRight(ARM_POSITION position) {

        stateTime.reset();
        if (position == ARM_POSITION.SAMPLE) {
            elbowMoveRight(ELBOW_SAMPLE);
            Logger.message("elbow to sample position");
            stateRight = ARM_STATE.MOVE_SAMPLE;
        } else if (position == ARM_POSITION.RUNG) {
            elbowMoveRight(ELBOW_RUNG);
            stateRight = ARM_STATE.MOVE_RUNG;
            Logger.message("elbow to rung position");
        } else if (position == ARM_POSITION.HIGH) {
            elbowMoveRight(ELBOW_UP_HIGH);
            stateRight = ARM_STATE.MOVE_HIGH;
            Logger.message("elbow to high position");
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMoveRight(ARM_IN);
            stateRight = ARM_STATE.MOVE_HOME;
            Logger.message("wrist and arm to home position");
        }
    }

    public void positionArmLeft(ARM_POSITION position) {

        if (position == ARM_POSITION.SAMPLE) {
            elbowMoveLeft(ELBOW_SAMPLE);
            opMode.sleep(500);
            armMoveLeft((ARM_SAMPLE));
            //wristMove(WRIST_DROP_LOW);
        } else if (position == ARM_POSITION.RUNG) {
            elbowMoveLeft(ELBOW_RUNG);
            opMode.sleep(500);
            elbowMoveLeft(ARM_RUNG);
        } else if (position == ARM_POSITION.HIGH) {
            elbowMoveLeft(ELBOW_UP_HIGH);
            armMoveLeft((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMoveLeft(ARM_IN);
            elbowMoveLeft(ELBOW_DOWN);
            while (leftArm.isBusy()) {
                if (!opMode.opModeIsActive()) {
                    break;
                }
            }
            leftArm.setPower(0);
        }
    }

    public void positionArmRight(ARM_POSITION position) {

        if (position == ARM_POSITION.SAMPLE) {
            elbowMoveRight(ELBOW_SAMPLE);
            opMode.sleep(500);
            armMoveRight((ARM_SAMPLE));
            //wristMove(WRIST_DROP_LOW);
        } else if (position == ARM_POSITION.RUNG) {
            elbowMoveRight(ELBOW_RUNG);
            opMode.sleep(500);
            elbowMoveRight(ARM_RUNG);
        } else if (position == ARM_POSITION.HIGH) {
            elbowMoveRight(ELBOW_UP_HIGH);
            armMoveRight((ARM_OUT_HIGH));
            //wristMove(WRIST_DROP_HIGH);
        } else if (position == ARM_POSITION.HOME) {
            //wristMove(WRIST_HOME);
            armMoveRight(ARM_IN);
            elbowMoveRight(ELBOW_DOWN);
            while (leftArm.isBusy()) {
                if (!opMode.opModeIsActive()) {
                    break;
                }
            }
            rightArm.setPower(0);
        }
    }

    public boolean positionCommand2 () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.a || gamepad.b || gamepad.x);
    }

    public boolean positionCommand1 () {
        Gamepad gamepad = opMode.gamepad1;
        return (gamepad.a || gamepad.b || gamepad.x);
    }

    public boolean dropCommand1 () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.right_trigger > 0);

    }

    public boolean dropCommand2 () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.right_trigger > 0);

    }

    public void displayControls(){
        opMode.telemetry.addLine("Right Arm Controls (Gamepad 2)\n" +
                        "  a - position arm at low position\n" +
                        "  b - position arm at mid position\n" +
                        "  x - position arm at high position\n" +
                        "  y - position arm at pickup position\n" +
                        "  right triggers - drop pixels\n" +
                        rightElbow.getCurrentPosition() + "\n"
                //"  left stick - move elbow (u/d)  arm (l/r)\n" +
                //"  right stick - manual rotate the hands\n"
        );
        opMode.telemetry.addLine("Left Arm Controls (Gamepad 1)\n" +
                        "  a - position arm at low position\n" +
                        "  b - position arm at mid position\n" +
                        "  x - position arm at high position\n" +
                        "  y - position arm at pickup position\n" +
                        "  right triggers - drop pixels\n" +
                        leftElbow.getCurrentPosition() + "\n"
                //"  left stick - move elbow (u/d)  arm (l/r)\n" +
                //"  right stick - manual rotate the hands\n"
        );
    }

    /**
     * Manually control the arms
     */
    public boolean control() {
        Gamepad gamepad2 = opMode.gamepad2;
        Gamepad gamepad1 = opMode.gamepad1;
        boolean handledLeft = true;
        boolean handledRight = true;

        run();   // ToDo remove when subclass from thread

        if (gamepad2.back) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad2.a) {
            // Move the arm to the lower drop position
            if (! aPressed2) {
                positionArmAsynLeft(ARM_POSITION.RUNG);
                positionArmAsynRight(ARM_POSITION.RUNG);
                aPressed2 = true;
            }
        } else {
            aPressed2 = false;
        }

        if (gamepad2.x) {
            // Move the arm to the middle drop position
            if (!xPressed2) {
                positionArmAsynLeft(ARM_POSITION.SAMPLE);
                positionArmAsynRight(ARM_POSITION.SAMPLE);
                xPressed2 = true;
            }
        } else {
            xPressed2 = false;
        }

        if (gamepad2.b) {
            // Move the arm to the higher drop position
            if (!bPressed2) {
                positionArmAsynLeft(ARM_POSITION.HIGH);
                positionArmAsynRight(ARM_POSITION.HIGH);
                bPressed2 = true;
            }
        } else {
            bPressed2 = false;
        }

        if (gamepad2.y) {
            // Move the arm to the home position, need to add button support
            if (!yPressed2) {
                positionArmAsynLeft(ARM_POSITION.HOME);
                positionArmAsynRight(ARM_POSITION.HOME);
                yPressed2 = true;
            }
        } else {
            yPressed2 = false;
        }

        if (gamepad2.left_stick_y != 0) {
            //msCt1 = System.currentTimeMillis();
            // manually move the elbow
            leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (true) {
                if (gamepad2.left_stick_y > 0) {
                    leftElbow.setPower(ELBOW_SPEED);
                    rightElbow.setPower(ELBOW_SPEED);
                } else if (gamepad2.left_stick_y < 0) {
                    leftElbow.setPower(-ELBOW_SPEED);
                    rightElbow.setPower(-ELBOW_SPEED);
                }
                lengthLeft = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                lengthRight = (rightArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                if (gamepad2.left_stick_y == 0 || lengthRight >= 30) {
                    while (lengthRight >= 30) {
                        rightElbow.setPower(-ELBOW_SPEED);
                        lengthRight = (rightArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                    }
                }
                if (gamepad2.left_stick_y == 0 || lengthLeft >= 30) {
                    while (lengthLeft >= 30) {
                        leftElbow.setPower(-ELBOW_SPEED);
                        lengthLeft = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                    }
                }
                break;
            }
            rightElbow.setPower(0);
            leftElbow.setPower(0);
            Logger.message("left elbow position %7d", leftElbow.getCurrentPosition());
            Logger.message("right elbow position %7d", rightElbow.getCurrentPosition());

        } else if (gamepad2.left_stick_x != 0) {
            // manually extend or retract the arm
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.left_stick_x > 0) {
                leftArm.setPower(ARM_SPEED);
                rightArm.setPower(ARM_SPEED);
            } else if (gamepad2.left_stick_x < 0) {
                leftArm.setPower(-ARM_SPEED);
                rightArm.setPower(-ARM_SPEED);
            }
            while (true) {
                lengthRight = (rightArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                lengthLeft = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                if (gamepad2.left_stick_y == 0 || lengthRight >= 30 || lengthLeft >= 30) {
                    while (lengthRight >= 30) {
                        rightElbow.setPower(-ELBOW_SPEED);
                        lengthRight = (rightArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                    }
                    while (lengthLeft >= 30) {
                        leftElbow.setPower(-ELBOW_SPEED);
                        lengthLeft = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            leftArm.setPower(0);
            rightArm.setPower(0);
            Logger.message( "left arm position %7d", leftArm.getCurrentPosition());
            Logger.message( "right arm position %7d", rightArm.getCurrentPosition());

        } else if (gamepad2.right_stick_y != 0) {
            // manually rotate the bucket
            /*while (gamepad2.right_stick_y != 0) {
                double position = wrist.getPosition();
                if (Double.isNaN(position))
                    position = WRIST_HOME;
                else if (gamepad2.right_stick_y > 0)
                    position += 0.0125;
                else if (gamepad2.right_stick_y < 0)
                    position -= 0.0125;
                wrist.setPosition(position);
                opMode.sleep(100);
            }
            Logger.message("wrist position %f", wrist.getPosition());
            wrist.setPower(gamepad2.right_stick_y);
            Logger.message("wrist position %f", wrist.getPower());*/

        } else {
            /*wrist.setPower(gamepad2.right_stick_y);
            handled = false;*/
        }

        /*if(gamepad2.right_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.REVERSE) {
                intake.setPower(1);
                iState = intakeStates.FORWARD;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        } else if(gamepad2.left_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.FORWARD) {
                intake.setPower(-1);
                iState = intakeStates.REVERSE;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        }*/

        return handledLeft || handledRight;
    }

    public boolean controlLeft() {
        Gamepad gamepad1 = opMode.gamepad1;
        boolean handledLeft = true;

        run();   // ToDo remove when subclass from thread

        if (gamepad1.back) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.a) {
            // Move the arm to the lower drop position
            if (! aPressed1) {
                //positionArmAsynLeft(ARM_POSITION.RUNG);
                aPressed1 = true;
            }
        } else {
            aPressed1 = false;
        }

        if (gamepad1.x) {
            // Move the arm to the middle drop position
            if (!xPressed1) {
                //positionArmAsynLeft(ARM_POSITION.SAMPLE);
                xPressed1 = true;
            }
        } else {
            xPressed1 = false;
        }

        if (gamepad1.b) {
            // Move the arm to the higher drop position
            if (!bPressed1) {
                //positionArmAsynLeft(ARM_POSITION.HIGH);
                bPressed1 = true;
            }
        } else {
            bPressed1 = false;
        }

        if (gamepad1.y) {
            // Move the arm to the home position, need to add button support
            if (!yPressed1) {
                positionArmAsynLeft(ARM_POSITION.HOME);
                yPressed1 = true;
            }
        } else {
            yPressed1 = false;
        }

        if (gamepad1.left_stick_y != 0) {
            //msCt1 = System.currentTimeMillis();
            // manually move the elbow
            leftElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (true) {
                if (gamepad1.left_stick_y > 0) {
                    leftElbow.setPower(ELBOW_SPEED);
                } else if (gamepad1.left_stick_y < 0) {
                    leftElbow.setPower(-ELBOW_SPEED);
                }
                lengthLeft = (leftArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                if (gamepad1.left_stick_y == 0 || lengthLeft >= 30) {
                    while (lengthLeft >= 30) {
                        leftElbow.setPower(-ELBOW_SPEED);
                        lengthLeft = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            leftElbow.setPower(0);
            Logger.message("left elbow position %7d", leftElbow.getCurrentPosition());

        } else if (gamepad1.left_stick_x != 0) {
            // manually extend or retract the arm
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.left_stick_x > 0) {
                leftArm.setPower(ARM_SPEED);
            } else if (gamepad1.left_stick_x < 0) {
                leftArm.setPower(-ARM_SPEED);
            }
            while (true) {
                lengthLeft = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                if (gamepad1.left_stick_x == 0 || lengthLeft >= 30) {
                    while (lengthLeft >= 30) {
                        leftArm.setPower(-ELBOW_SPEED);
                        lengthLeft = (leftArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(leftElbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            int positionLeft = leftArm.getCurrentPosition();
            leftArm.setPower(0);
            /*leftArm.setTargetPosition(positionLeft);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            Logger.message( "left arm position %7d", positionLeft);

        /*} else if (gamepad1.right_stick_y != 0) {
        } else if (gamepad1.right_stick_x != 0) {*/
        } else {
            //wrist.setPower(gamepad2.right_stick_y);
            //handled = false;
        }

        /*right joystick left and right moves 1st servo (vel)
        right y - 2nd servo l & r (accel)
        right_trigger controls specimen (jerk, but mostly known as specimenLeft)
         */

        /*if(gamepad2.right_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.REVERSE) {
                intake.setPower(1);
                iState = intakeStates.FORWARD;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        } else if(gamepad2.left_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.FORWARD) {
                intake.setPower(-1);
                iState = intakeStates.REVERSE;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        }*/

        return handledLeft;
    }

    public boolean controlRight() {
        Gamepad gamepad2 = opMode.gamepad2;
        boolean handledRight = true;

        run();   // remove when subclass from thread

        if (gamepad2.back) {
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad2.a) {
            // Move the arm to the lower drop position
            if (! aPressed2) {
                positionArmAsynRight(ARM_POSITION.RUNG);
                aPressed2 = true;
            }
        } else {
            aPressed2 = false;
        }

        if (gamepad2.x) {
            // Move the arm to the middle drop position
            if (!xPressed2) {
                positionArmAsynRight(ARM_POSITION.SAMPLE);
                xPressed2 = true;
            }
        } else {
            xPressed2 = false;
        }

        if (gamepad2.b) {
            // Move the arm to the higher drop position
            if (!bPressed2) {
                positionArmAsynRight(ARM_POSITION.HIGH);
                bPressed2 = true;
            }
        } else {
            bPressed2 = false;
        }

        if (gamepad2.y) {
            // Move the arm to the home position, need to add button support
            if (!yPressed2) {
                positionArmAsynRight(ARM_POSITION.HOME);
                yPressed2 = true;
            }
        } else {
            yPressed2 = false;
        }

        if (gamepad2.left_stick_y != 0) {
            //msCt1 = System.currentTimeMillis();
            // manually move the elbow
            rightElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (true) {
                if (gamepad2.left_stick_y > 0) {
                    rightElbow.setPower(ELBOW_SPEED);
                } else if (gamepad2.left_stick_y < 0) {
                    rightElbow.setPower(-ELBOW_SPEED);
                }
                lengthRight = (rightArm.getCurrentPosition()/encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                if (gamepad2.left_stick_y == 0 || lengthRight >= 30) {
                    while (lengthRight >= 30) {
                        rightElbow.setPower(-ELBOW_SPEED);
                        lengthRight = (rightArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            rightElbow.setPower(0);
            Logger.message("right elbow position %7d", rightElbow.getCurrentPosition());

        } else if (gamepad2.left_stick_x != 0) {
            // manually extend or retract the arm
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.left_stick_x > 0) {
                rightArm.setPower(ARM_SPEED);
            } else if (gamepad2.left_stick_x < 0) {
                rightArm.setPower(-ARM_SPEED);
            }
            while (true) {
                lengthRight = (rightArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                if (gamepad2.left_stick_x == 0 || lengthRight >= 30) {
                    Logger.message("LengthRight:" + lengthRight);
                    Logger.message("Left Stick X:" + gamepad2.left_stick_x);
                    while (lengthRight >= 30) {
                        Logger.message("lengthProb");
                        rightArm.setPower(-ELBOW_SPEED);
                        lengthRight = (rightArm.getCurrentPosition() / encoderInch + 10.5) * cos(Math.toRadians(rightElbow.getCurrentPosition() / encoderDegree));
                    }
                    break;
                }
            }
            int positionRight = rightArm.getCurrentPosition();
            rightArm.setPower(0);
            /*rightArm.setTargetPosition(positionRight);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            Logger.message( "left arm position %7d", positionRight);
        }
        if (gamepad2.right_stick_y != 0) {
            // manually rotate accel
            double positionA = 0.5;
            if (gamepad2.right_stick_y > 0) {
                positionA = 1;
            } else if (gamepad2.right_stick_y < 0) {
                positionA = -1;
            }
            accel.setPosition(positionA);
            Logger.message("accel position %f", accel.getPosition());
            /*accel.setPower(gamepad1.right_stick_y);
            Logger.message("accel position %f", accel.getPower());*/
        } else {
            accel.setPosition(0.5);
        }
        if (gamepad2.right_stick_x != 0) {
            double positionV = 0.5;
            if (Double.isNaN(positionV))
                positionV = WRIST_HOME;
            else if (gamepad2.right_stick_x > 0)
                positionV = 1;
            else if (gamepad2.right_stick_x < 0)
                positionV = -1;
            vel.setPosition(positionV);
            Logger.message("vel position %f", vel.getPosition());
        } else {
            vel.setPosition(0.5);
        }

        /*if(gamepad2.right_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.REVERSE) {
                intake.setPower(1);
                iState = intakeStates.FORWARD;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }
        } else if(gamepad2.left_bumper) {
            if (iState == intakeStates.OFF || iState == intakeStates.FORWARD) {
                intake.setPower(-1);
                iState = intakeStates.REVERSE;
            } else {
                intake.setPower(0);
                iState = intakeStates.OFF;
            }*/
        return handledRight;
    }
}