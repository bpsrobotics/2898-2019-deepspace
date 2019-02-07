package com.team2898.engine.subsystems

import com.kauailabs.navx.frc.AHRS
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.signalProcessing.MovingAverageFilter
import com.team2898.engine.types.CircularArray
import com.team2898.engine.types.Timestamp
import com.team2898.robot.config.NavxConf.*
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.RunEvery
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as smartDashboard


object Navx : ISelfCheck, ILooper {
    override val loop: AsyncLooper
        get() = AsyncLooper(20.0) {
            smartDashboard.putNumber("Navx yaw", yaw)
            smartDashboard.putNumber("Navx yaw rate", yawRate)
            smartDashboard.putNumber("Navx rotation", rotation.degrees)
        }


    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val navx = AHRS(NAVX_PORT, 100)

    // in degrees
    val yaw: Double
        get() = navx.yaw.toDouble()
    // in degrees/second
    val yawRate: Double
        get() = navx.rate
    val rotation: Rotation2d
        get() = Rotation2d.createFromDegrees(yaw)


    @Synchronized
    fun reset() {
        navx.reset()
    }

    override fun selfCheckup(): Boolean
            = navx.isConnected

    override fun selfTest(): Boolean = selfCheckup()
}
