//package com.team2898.robot.motion.pathfinder
//
//import com.team2898.engine.logging.LogLevel
//import com.team2898.engine.logging.Logger
//import com.team2898.engine.logging.reflectLocation
//import com.team2898.engine.motion.pathfinder.ProfileSettings
//import com.team2898.engine.motion.pathfinder.config.FILE_EXT
//import com.team2898.engine.motion.pathfinder.config.FILE_PATH
//import com.team2898.engine.motion.pathfinder.tripleToWaypoint
//import edu.wpi.first.wpilibj.Timer
//import jaci.pathfinder.Pathfinder
//import jaci.pathfinder.Trajectory
//import jaci.pathfinder.modifiers.TankModifier
//import kotlinx.serialization.ImplicitReflectionSerializer
//import kotlinx.serialization.json.JSON
//import kotlinx.serialization.stringify
//import java.io.File
//import java.security.MessageDigest
//import kotlin.system.measureTimeMillis
//
//object ProfileGenerator {
//    @ImplicitReflectionSerializer
//    fun genUnmodifiedProfile(profile: ProfileSettings): Trajectory {
//        val serialized = JSON().stringify(profile)
//        val file = File("$FILE_PATH/${serialized.md5()}.$FILE_EXT")
//        val logInfo = Logger.logInfo(reflectLocation(), LogLevel.INFO, "Starting profile generation, hash of ${serialized.md5()}")
//
//        val mprofile: Trajectory
//        if (file.exists()) {
//            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Pregenerated profile found!")
//            mprofile = Pathfinder.readFromCSV(file)
//        } else {
//            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Pregenerated profile not found, generating!")
//            println(file.absolutePath)
//            val start = Timer.getFPGATimestamp()
//            val path = tripleToWaypoint(profile.wayPoints)
//            val config = Trajectory.Config(
//                    profile.fitMethod,
//                    profile.sampleRate,
//                    1.0 / profile.hz,
//                    profile.maxVel,
//                    profile.maxAcc,
//                    profile.maxJerk
//            )
//            println("check point")
//            mprofile = Pathfinder.generate(path, config)
//            println("check point")
//            Pathfinder.writeToCSV(file, mprofile)
//            println("check point")
//            val dt = Timer.getFPGATimestamp() - start
//            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Generated profile in ${"%.3f".format(dt)} seconds")
//        }
//        return mprofile
//    }
//
//    @ImplicitReflectionSerializer
//    fun genMotifiedProfile(profile: ProfileSettings): Pair<Trajectory, Trajectory> {
//        val prof = genUnmodifiedProfile(profile)
//        val modified = TankModifier(prof).modify(profile.wheelbaseWidth)
//        return Pair<Trajectory, Trajectory>(modified.leftTrajectory, modified.rightTrajectory)
//    }
//
//    fun String.md5(): String {
//        val md = MessageDigest.getInstance("MD5")
//        val digested = md.digest(toByteArray())
//        return digested.joinToString("") {
//            String.format("%02x", it)
//        }
//    }
//}