package com.team2898.engine.subsystems

import com.team2898.engine.math.clamp
import com.team2898.engine.math.linear.*
import com.team2898.robot.config.ArmConf.*
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix

abstract class SingleJointedArmLQR {
    val A = Arm_A
    val B = Arm_B
    val Kff = Arm_Kff
    val Kc = Arm_Kc
    val M = Arm_M

    val dt = 0.01

    fun clampU(u: Double) = clamp(u, 12.0)

    val C = Matrix(arrayOf(row(1.0, 0.0)))
    val D = Matrix(arrayOf(row(0.0)))

    var x = Matrix(arrayOf(row(0.0), row(0.0))).T

    var xHat = x

    var u = Matrix(arrayOf(row(0.0))).T
        set(value) {
            value[0, 0] = clampU(value[0, 0])
            field = value
        }

    val A_inv
        get() = MatrixUtils.inverse(A)


    val kalmanGain = Arm_M
    val L = A * kalmanGain


    fun correctObserver() {
        xHat += A_inv * L * (C * x - C * xHat - D * u)
    }

    fun predictObserver() {
        xHat = A * xHat + B * u
    }

    fun genU(r: Matrix, ff: Boolean = true, x: RealMatrix = xHat): Matrix {
        predictObserver()
        val u_feedback = Matrix((Kc * (r - x)).data)
        if (!ff) return u_feedback

        val u_feedforward = Kff * (r - A * r)
        val u = Matrix((u_feedback + u_feedforward).data)
        return u
    }
}
