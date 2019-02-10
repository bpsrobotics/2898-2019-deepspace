package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.engine.subsystems.DrivetrainLQR
import com.team2898.robot.config.LEFT_MASTER
import com.team2898.robot.config.LEFT_SLAVE
import com.team2898.robot.config.RIGHT_MASTER
import com.team2898.robot.config.RIGHT_SLAVE
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import org.apache.commons.math3.linear.RealMatrix
import kotlin.math.PI


object Drivetrain: DrivetrainLQR() {
    override val wheelbase: Double = 2.0

//    override val A = Dt_A
//    override val B = Dt_B
//    override val Kc = Dt_Kc
//    override val Kff = Dt_Kff
//    override val M = Dt_M


    val leftEnc = Encoder(0, 1)
    val rightEnc = Encoder(2, 3)

    var prevTime = Timer.getFPGATimestamp()
    var prevDist = Pair(0.0, 0.0)

    val leftMaster = TalonWrapper(LEFT_MASTER)
    val leftSlave = TalonWrapper(LEFT_SLAVE)

    val rightMaster = TalonWrapper(RIGHT_MASTER)
    val rightSlave = TalonWrapper(RIGHT_SLAVE)


    init {
        listOf(leftEnc, rightEnc).forEach {
            it.apply {
                distancePerPulse = 6 * PI / 256 / 12
            }
        }
        rightEnc.setReverseDirection(true)

        rightSlave slaveTo rightMaster
        leftSlave slaveTo leftMaster
        listOf(leftMaster, rightMaster).forEach {
            it.apply {
            }
        }
        AsyncLooper(50.0) {
            val leftVel = (leftEnc.distance - prevDist.first) / (Timer.getFPGATimestamp() - prevTime)
            val rightVel = (rightEnc.distance - prevDist.second) / (Timer.getFPGATimestamp() - prevTime)
            prevTime = Timer.getFPGATimestamp()
            prevDist = Pair(leftEnc.distance, rightEnc.distance)
            x = Matrix(arrayOf(row(leftVel, rightVel))).T
            correctObserver()
        }.start()
    }

    fun openLoopPower(driveSignal: DriveSignal) {
        leftMaster.set(ControlMode.PercentOutput, driveSignal.left)
        rightMaster.set(ControlMode.PercentOutput, driveSignal.right)
    }

    fun masters(block: TalonWrapper.() -> Unit) {
        listOf(leftMaster, rightMaster).forEach {
            it.apply(block)
        }
    }

    fun encoders(block: Encoder.() -> Unit) {
        listOf(leftEnc, rightEnc).forEach {
            it.apply(block)
        }
    }
}
