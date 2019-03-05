package com.team2898.engine.subsystems


import com.team2898.engine.math.clamp
import com.team2898.engine.math.linear.*
import com.team2898.robot.config.ArmConf.*
import com.team2898.robot.config.DrivetrainConf.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.apache.commons.math3.linear.MatrixUtils
import org.apache.commons.math3.linear.RealMatrix

abstract class DrivetrainLQR {
    val A = Dt_A
    val B = Dt_B
    val Kff = Dt_Kff
    val Kc = Dt_Kc
    val M = Dt_M

    val dt = 0.01

    fun clampU(u: Double) = clamp(u, 12.0)

    val C = Matrix(arrayOf(row(1.0, 0.0), row(0.0, 1.0)))
    val D = Matrix(arrayOf(row(0.0, 0.0), row(0.0, 0.0)))

    var x = Matrix(arrayOf(row(0.0, 0.0))).T
    var y = Matrix(arrayOf(row(0.0, 0.0))).T


    var xHat = x

    var u = Matrix(arrayOf(row(0.0, 0.0))).T
        set(value) {
            value[0, 0] = clampU(value[0, 0])
            value[1, 0] = clampU(value[1, 0])
            field = value
        }


    val kalmanGain = M


    fun correctObserver() {
//        xHat += A_inv * L * (C * x - C * xHat - D * u)
        xHat += kalmanGain * (y - C * xHat - D * u)
    }

    fun predictObserver() {
        xHat = A * xHat + B * u
    }

    fun updatePlant() {
        y = C * x + D * u
    }

    fun genU(r_ft: Matrix, ff: Boolean = true, x: RealMatrix = xHat): Matrix {
        val r = r_ft.scalarMultiply(0.3048)
        val xM = x.scalarMultiply(0.3048)
//        updatePlant()
        correctObserver()
        val u_feedback = Matrix((Kc * (r - xM)).data)
        if (!ff) return u_feedback

        val u_feedforward = Kff * (r - A * r)
        val uReturn = Matrix((u_feedback + u_feedforward).data)
        predictObserver()
        u = uReturn
        return uReturn
    }
}
