package com.team2898.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.NotifierLooper
import com.team2898.engine.math.linear.*
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


object Drivetrain: DrivetrainLQR() {
    val wheelbase: Double = 2.0

//    val leftEnc = Encoder(0, 1)
//    val rightEnc = Encoder(2, 3)

    var prevTime = Timer.getFPGATimestamp()
    var prevDist = Pair(0.0, 0.0)

    val leftMaster = TalonWrapper(LEFT_MASTER)
    val leftSlave = TalonWrapper(LEFT_SLAVE)

    val rightMaster = TalonWrapper(RIGHT_MASTER)
    val rightSlave = TalonWrapper(RIGHT_SLAVE)

    var r = Matrix(arrayOf(row(0.0, 0.0))).T


    init {

        x = Matrix(arrayOf(row(0.0, 0.0))).T
//        listOf(leftEnc, rightEnc).forEach {
//            it.apply {
//                distancePerPulse = 6 * PI / 256 / 12
//            }
//        }
//        rightEnc.setReverseDirection(true)

        rightSlave slaveTo rightMaster
        leftSlave slaveTo leftMaster
        listOf(leftMaster, rightMaster).forEach {
            it.apply {
                configVoltageCompSaturation(12.0, 10)
            }
        }
//        AsyncLooper(100.0) {
//            val leftVel = (leftEnc.distance - prevDist.first) / (Timer.getFPGATimestamp() - prevTime)
//            val rightVel = (rightEnc.distance - prevDist.second) / (Timer.getFPGATimestamp() - prevTime)
//            prevTime = Timer.getFPGATimestamp()
//            prevDist = Pair(leftEnc.distance, rightEnc.distance)
//            x = Matrix(arrayOf(row(leftVel, rightVel))).T
//            SmartDashboard.putNumber("left vel", x[0, 0])
//            SmartDashboard.putNumber("right vel", x[1, 0])
//            SmartDashboard.putNumber("left U", u[0, 0])
//            SmartDashboard.putNumber("right U", u[1, 0])
//        }.start()
    }

    val looper = NotifierLooper(100.0) {
        openLoopPower(genU(Matrix(r.data), x = x))
    }

    fun openLoopPower(driveSignal: DriveSignal) {
        leftMaster.set(ControlMode.PercentOutput, driveSignal.left) // change the sign if left is going back
        rightMaster.set(ControlMode.PercentOutput, -driveSignal.right) // change the sign if right is going back
    }
    fun openLoopPower(vels: RealMatrix) {
        vels[0, 0] = clampU(vels[0, 0])
        vels[1, 0] = clampU(vels[1, 0])
        openLoopPower(driveSignal = DriveSignal(vels[0, 0]/12, vels[1, 0]/12))
    }

    fun masters(block: TalonWrapper.() -> Unit) {
        listOf(leftMaster, rightMaster).forEach {
            it.apply(block)
        }
    }

    fun encoders(block: Encoder.() -> Unit) {
//        listOf(leftEnc, rightEnc).forEach {
//            it.apply(block)
//        }
    }
}
