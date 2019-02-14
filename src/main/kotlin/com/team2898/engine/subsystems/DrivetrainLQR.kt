package com.team2898.engine.subsystems

import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.clamp
import com.team2898.engine.math.linear.*
import com.team2898.robot.config.DrivetrainConf.*
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix

abstract class DrivetrainLQR {
    abstract val wheelbase: Double
    val A: Matrix = Dt_A
    val B: Matrix = Dt_B
    val Kc: Matrix = Dt_Kc
    val Kff: Matrix = Dt_Kff
    val M: Matrix = Dt_M
// TODO move this somewhere

    var history = listOf<dtState>()

    data class dtState(var pose: RigidTransform2d, var vel: Twist2d, var time: Double)

    var state = dtState(pose = RigidTransform2d(
            rotation = Rotation2d(1.0, 0.0),
            translation = Translation2d(0.0, 0.0)
    ), vel = Twist2d(0.0, 0.0, 0.0), time = 0.0)

    fun clampU(u: Double) = clamp(u, 12.0)

    val dt = 0.01

    val C = Matrix(arrayOf(
            row(1.0, 0.0),
            row(0.0, 1.0)
    ))

    val D = Matrix(arrayOf(
            row(0.0, 0.0),
            row(0.0, 0.0)
    ))

    // leftSpeed, rightSpeed
    var x: RealMatrix = Matrix(arrayOf(row(0.0, 0.0))).T

    var xHat = x

    // leftVoltage, rightVoltage
    var u = Matrix(arrayOf(row(0.0, 0.0))).T
        set(value) {
            value[0, 0] = clampU(value[0, 0])
            value[1, 0] = clampU(value[1, 0])
            field = value
        }

    fun step(u: Matrix): dtState {
        this.u = u
        return step()
    }

    fun step(): dtState {
        correctObserver()

        val vels = Pair(x[0, 0], x[1, 0]) // * 3.281 Uncomment this if using M in encoder value
        val v = (vels.first + vels.second) / 2
        val w = (vels.second - vels.first) / wheelbase

        state = state.copy(
                pose = state.pose.transformBy(RigidTransform2d.fromDelta(Twist2d(v, 0.0, w) * dt)),
                vel = Twist2d(v, 0.0, w),
                time = state.time + dt
        )
        history += state.copy()

        predictObserver()
//        x = A * x + B * u
        return state
    }

    val A_inv
        get() = MatrixUtils.inverse(A)

    fun correctObserver() {
        xHat += A_inv * L * (C * x - C * xHat - D * u)
    }

    fun predictObserver() {
        xHat = A * xHat + B * u
    }

    val kalman_gain = M

    val L = A * kalman_gain

    fun genU(r: Matrix, ff: Boolean = true, x: RealMatrix = xHat): Matrix {
        val r = r.scalarMultiply(1 / 3.281) // conf ft -> m
        val u_feedback = Matrix((Kc * (r - x)).data)
        if (!ff) return u_feedback

        val u_feedforward = Kff * (r - A * r)
        val u = Matrix((u_feedback + u_feedforward).data)
        return u
    }
}