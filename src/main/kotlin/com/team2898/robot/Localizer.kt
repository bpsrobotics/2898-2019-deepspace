//package com.team2898.robot
//
//import com.team2898.engine.async.NotifierLooper
//import com.team2898.engine.subsystems.Navx
//import com.team2898.robot.subsystem.Drivetrain
//import org.ghrobotics.lib.localization.TankEncoderLocalization
//import org.ghrobotics.lib.mathematics.units.degree
//import org.ghrobotics.lib.mathematics.units.feet
//
//object Localizer {
//    val dt = 0.01
//
////    val localization = TankEncoderLocalization({Navx.yaw.degree}, {Drivetrain.leftEnc.distance.feet}, {Drivetrain.rightEnc.distance.feet})
//
//    init {
//        NotifierLooper(1/dt) {
////            localization.update()
//        }
//    }
//}