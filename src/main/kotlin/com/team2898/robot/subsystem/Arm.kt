package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.team254.lib.util.motion.*
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.NotifierLooper
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.TalonWrapper
import com.team2898.engine.subsystems.SingleJointedArmLQR
import com.team2898.robot.config.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos

object Arm : SingleJointedArmLQR() {
    private val armEnc = Encoder(ARM_ENC_A, ARM_ENC_B)
    private var prevDist = 0.0
    private var prevTime = 0.0
    private var prevVel = 0.0
    private var acc = 0.0
    private val constrains = MotionProfileConstraints(3.0, 1.0)

    private val leftMaster = TalonWrapper(ARM_LEFT_MASTER)
    private val leftSlave = TalonWrapper(ARM_LEFT_SLAVE)
    private val rightMaster = TalonWrapper(ARM_RIGHT_MASTER)
    private val rightSlave = TalonWrapper(ARM_RIGHT_SLAVE)

    val brake: Boolean
        get() {
            if (isZeroing) return false
            return Timer.getFPGATimestamp() >= (profile?.endTime() ?: return true)
        }

    private var isZeroing = false
    private var target: Double = 0.0

    private val safeRange = -0.2..2.24

    private val limitSwitch = DigitalInput(SAFETY_LIMIT_PORT)

    private var limitSwitchLeadingEdge: Boolean = false
        get() {
            val v = !limitSwitch.get()
            val o = !field && v
            field = v
            return o
        }
        set(_) {}

    fun startZeroing() {
        panic = false
        isZeroing = true
        leftMaster.set(0.2)
    }

    // TODO: this is allocation-heavy, make a mutable version?
    val currentPos: MotionState
        get() = MotionState(
                Timer.getFPGATimestamp(),
                x[0, 0],
                x[1, 0],
                acc
        )


    private var profile: MotionProfile? = null

    private var panic = false

    init {

        armEnc.distancePerPulse = ((22 / 60.0) * 2 * PI) / 600.0 // radians

        leftSlave slaveTo leftMaster
        rightMaster slaveTo leftMaster
        rightSlave slaveTo leftMaster


        listOf(leftMaster, leftSlave, rightSlave, rightMaster).forEach {
            it.apply {
                configContinuousCurrentLimit(15)
                configPeakCurrentLimit(20)
                configVoltageCompSaturation(12.0)
                setNeutralMode(NeutralMode.Brake)
            }
        }

        AsyncLooper(100.0) {
            val armVel = armEnc.rate
            acc = (armVel - prevVel) / (Timer.getFPGATimestamp() - prevTime)
            prevTime = Timer.getFPGATimestamp()
            prevDist = armEnc.distance
            x = Matrix(arrayOf(row(armEnc.distance, armVel))).T
        }.start()

//        val a = DigitalInput(8)
//        val b = DigitalInput(9)

        AsyncLooper(50.0) {
            SmartDashboard.putNumber("arm pos", x[0, 0] + ARM_OFFSET)
//            SmartDashboard.putNumber("arm pos raw", x[0, 0])
            SmartDashboard.putNumber("arm vel", x[1, 0])
            SmartDashboard.putNumber("arm dist raw", armEnc.distance)
//            SmartDashboard.putNumber("lm talon current", leftMaster.statorCurrent)
//            SmartDashboard.putNumber("ls talon current", leftSlave.statorCurrent)
//            SmartDashboard.putNumber("rm talon current", rightMaster.statorCurrent)
//            SmartDashboard.putNumber("rs talon current", rightSlave.statorCurrent)
//            SmartDashboard.putNumber("Kf", -ARM_KF * cos(x[0, 0] + ARM_OFFSET))
            SmartDashboard.putBoolean("arm limit switch", !limitSwitch.get())

//            SmartDashboard.putBoolean("aa", a.get())
//            SmartDashboard.putBoolean("ab", b.get())
        }.start()

        var encoderUnpluggedPos = -1
        var notMovingTicks = -1

        // Control Loop
        NotifierLooper(100.0) {
            if (limitSwitchLeadingEdge || (!limitSwitch.get() && isZeroing)) {
                armEnc.reset()
                isZeroing = false
                println("Resetting arm encoder")
            }

            if (panic) {
                println("ENCODER ERROR DETECTED")
                profile = null
                leftMaster.set(0.0)
                rightMaster.set(0.0)
                return@NotifierLooper
            }

            if (notMovingTicks > 20) {
                profile = null
                panic = true
                leftMaster.set(0.0)
                rightMaster.set(0.0)
                notMovingTicks = 0
                DriverStation.reportError("ENCODER UNPLUGGED", false)
            }
            if (leftMaster.motorOutputPercent.absoluteValue > 0.1) {
                if ((armEnc.get() - encoderUnpluggedPos).absoluteValue < 2) {
                    notMovingTicks++
                } else {
                    notMovingTicks = 0
                    encoderUnpluggedPos = armEnc.get()
                }
            } else {
                notMovingTicks = 0
                encoderUnpluggedPos = armEnc.get()
            }

            if (isZeroing) return@NotifierLooper

            if (armEnc.distance !in safeRange) {
                profile = null
                println("SAFETY LIMIT")
            }

            val currentTime = Timer.getFPGATimestamp()
            val state = profile?.stateByTimeClamped(currentTime) ?: run {
                leftMaster.set(0.0)
                return@NotifierLooper
            }
            SmartDashboard.putNumber("arm target p", state.pos() + ARM_OFFSET)
            SmartDashboard.putNumber("arm target v", state.vel())

            // TODO: allocating two matrices and an array in a loop is kinda bad
            val r = Matrix(Matrix(arrayOf(row(state.pos(), state.vel()))).T.data)

            val voltage = Arm.genU(r)[0, 0]
            val outPutVoltage = voltage + (ARM_KF * cos(x[0, 0] - ARM_OFFSET))

            leftMaster.set(ControlMode.PercentOutput, if (!brake) {
                clampU(-outPutVoltage / 12)
            } else {
                0.0
            })
        }.start()
    }

    fun updatePosition(target: ArmPositions) {
        updateTarget(target.p)
    }

    fun updateTarget(targetPos: Double) {
        if (target == targetPos) return
        target = targetPos
        profile = MotionProfileGenerator.generateProfile(constrains, MotionProfileGoal(targetPos), currentPos)
    }

    /* note: most of these names are guessed */
    enum class ArmPositions(val p: Double) {
        BOTTOM(0.0),
        CARGO_C(cargo), CARGO_R1(cargol1), CARGO_R2(cargol2 + 0.1), CARGO_R3(cargol3),
        HATCH_R1(hatchl1 + 0.45), HATCH_R2(hatchl2 + 0.45), HATCH_R3(hatchl3 + 0.25)
    }
}