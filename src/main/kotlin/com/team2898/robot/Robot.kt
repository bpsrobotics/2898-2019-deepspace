package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.drivers.I2C_SLAVE_DEVICE_ADDRESS
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.commands.Safety
import com.team2898.robot.commands.commandGroups.TeleopWSafety
import com.team2898.robot.subsystem.*
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.serialization.ImplicitReflectionSerializer

@ImplicitReflectionSerializer
object Robot : TimedRobot() {
    val i2c = I2C(I2C.Port.kOnboard, I2C_SLAVE_DEVICE_ADDRESS)
    val data = byteArrayOf()
    override fun robotInit() {
        val LED = DigitalOutput(2)
        AsyncLooper(50.0) {
            if (OI.driverController.getRawButton(5)) LED.set(true)
            else LED.set(false)
        }.start()
        CameraServer.getInstance().startAutomaticCapture()
        Navx.reset()
        Arm
        Drivetrain
        DiscBrake
        Intake
        Safety
    }
    override fun autonomousInit() {
        TeleopWSafety.start()
    }

    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        TeleopWSafety.start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
//        DiscBrake.brakeUpdate(false)
    }

    override fun disabledPeriodic() {
//        DiscBrake.brakeUpdate(false)
    }
}

@ImplicitReflectionSerializer
fun main() {
    RobotBase.startRobot { Robot }
}