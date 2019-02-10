package com.team2898.robot.subsystem

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.engine.subsystems.SingleJointedArmLQR
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI

object Arm : SingleJointedArmLQR() {
    val armEnc = Encoder(3, 4)
    var prevDist = 0.0
    var prevTime = 0.0

    init {
        armEnc.apply {
            distancePerPulse = 22 / 60 / 600 * 2 * PI // rad
        }

        AsyncLooper(50.0) {
            val armVel = (armEnc.distance - prevDist) / (Timer.getFPGATimestamp() - prevTime)
            prevTime = Timer.getFPGATimestamp()
            prevDist = armEnc.distance
            x = Matrix(arrayOf(row(armEnc.distance), row(armVel)))
            correctObserver()
        }.start()

        AsyncLooper(50.0) {
            SmartDashboard.putNumber("arm pos", x[0, 0])
            SmartDashboard.putNumber("arm vel", x[0, 1])
        }.start()
    }
}