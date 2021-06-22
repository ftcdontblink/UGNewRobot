package org.firstinspires.ftc.teamcode.opmodes

import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.*

internal class KtDevPipeline : OpenCvPipeline() {
    enum class Color {
        BLUE, RED
    }

    companion object {
        @JvmField
        var FOCAL_LENGTH = 1010.7 / (720.0 / 360.0) * (1.25)
        @JvmField
        var CENTER_X = 638.2174071943463 / (720.0 / 360.0)
        @JvmField
        var CENTER_Y = 340.2355228022204 / (720.0 / 360.0)
        var DISTORTION_PARAMS = MatOfDouble(0.2126095014133333 * 0.64, -1.001796829192392 * 0.64 * 0.64, 0.000504850246603286 * 0.64, -0.0001913578573509387 * 0.64, 1.419425306492814 * 0.64 * 0.64 * 0.64)

        @JvmField
        var HSV_LOWER_BLUE = Scalar(100.0, 100.0, 100.0)
        @JvmField
        var HSV_UPPER_BLUE = Scalar(120.0, 256.0, 256.0)
        @JvmField
        var HSV_LOWER_RED = Scalar(170.0, 100.0, 100.0)
        @JvmField
        var HSV_UPPER_RED = Scalar(10.0, 256.0, 256.0)

        // tl, bl, tr, br
        var SLOT_POINTS = MatOfPoint3f(
                Point3(0.05, 0.0, 0.0), Point3(-0.05, 5.5, -0.3),
                Point3(15.95, 0.0, 0.0), Point3(16.05, 5.5, -0.3)
        )

        @JvmField
        var MIN_GOAL_WIDTH = 60.0

        @JvmField
        var MIN_SLOT_WIDTH = 20.0
        @JvmField
        var MAX_SLOT_WIDTH = 250.0

        // width / height ratio
        @JvmField
        var MIN_SLOT_RATIO = 2.0
        @JvmField
        var MAX_SLOT_RATIO = 5.0

        // static so they're controllable through dashboard
        @JvmField
        var stageNum = 0
        @JvmField
        var color = Color.BLUE
    }

    private val filteredMat = Mat()
    private val ycrcbMat = Mat()
    private val yMat = Mat()
    private val crMat = Mat()
    private val cColorMat = Mat()
    private val hsvMat = Mat()
    private val rangeMat = Mat()
    private val range1Mat = Mat()
    private val range2Mat = Mat()
    private val denoisedRangeMat = Mat()
    private val goalContourMat = Mat()
    private var convexGoalMat = Mat()
    private var slotMat = Mat()
    private var denoisedSlotMat = Mat()
    private var selectedSlotEdgeMat = Mat()
    private val slotContourMat = Mat()
    private var selectedSlotMat = Mat()
    private var dilatedSelectedSlotMat = Mat()
    private val finalMat = Mat()
    private val coolMat = Mat()

    private val hierarchy = Mat()
    private val goalContours: MutableList<MatOfPoint> = ArrayList()

    private val hullIndices = MatOfInt()

    private val slotContours: MutableList<MatOfPoint> = ArrayList()

    private val contour2f = MatOfPoint2f()
    private val approx2f = MatOfPoint2f()
    private val approx = MatOfPoint()

    private val corners = MatOfPoint()
    private val corners2f = MatOfPoint2f()

    private val rvec = Mat()
    private val tvec = Mat()

    private val cameraData = doubleArrayOf(FOCAL_LENGTH, 0.0, CENTER_X, 0.0, FOCAL_LENGTH, CENTER_Y, 0.0, 0.0, 1.0)
    private val camera = Mat(3, 3, CvType.CV_32FC1).apply { put(0, 0, *cameraData) }

    private val wideElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 3.0))
    private val normalElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))

    fun setColor(color: Color) {
        Companion.color = color
    }

    override fun onViewportTapped() {
        stageNum++
    }

    override fun processFrame(input: Mat): Mat {
        Core.multiply(input, Scalar(0.3, 0.3, 0.3), finalMat)
        input.copyTo(goalContourMat)
        input.copyTo(slotContourMat)

        if (convexGoalMat.size() != input.size()) {
            convexGoalMat.release()
            convexGoalMat = Mat.zeros(input.size(), CvType.CV_8UC1)
        } else {
            convexGoalMat.setTo(Scalar(0.0))
        }

        if (selectedSlotMat.size() != input.size()) {
            selectedSlotMat.release()
            selectedSlotMat = Mat.zeros(input.size(), CvType.CV_8UC1)
        } else {
            selectedSlotMat.setTo(Scalar(0.0))
        }

        selectedSlotEdgeMat.setTo(Scalar(0.0))

        // initial noise reduction, bilateral is good for noise reduction while preserving boundaries
//        Imgproc.bilateralFilter(input, filteredMat, 5, 100, 100);
        input.copyTo(filteredMat)

        // rgb -> ycrcb
        Imgproc.cvtColor(filteredMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        // rgb -> hsv
        Imgproc.cvtColor(filteredMat, hsvMat, Imgproc.COLOR_RGB2HSV)

        // extract cr or cb channel based on color
        Core.extractChannel(ycrcbMat, cColorMat, if (color == Color.BLUE) 2 else 1)
        val hsvLower = if (color == Color.BLUE) HSV_LOWER_BLUE else HSV_LOWER_RED
        val hsvUpper = if (color == Color.BLUE) HSV_UPPER_BLUE else HSV_UPPER_RED

        // hsv filtering
        if (hsvLower.`val`[0] > hsvUpper.`val`[0]) {
            // this "wraps" the hue range (channel 0) into a two part range on both extremes of the h spectrum
            Core.inRange(hsvMat, hsvLower, Scalar(181.0, hsvUpper.`val`[1], hsvUpper.`val`[2]), range1Mat)
            Core.inRange(hsvMat, Scalar(0.0, hsvLower.`val`[1], hsvLower.`val`[2]), hsvUpper, range2Mat)
            Core.bitwise_or(range1Mat, range2Mat, rangeMat)
        } else {
            Core.inRange(hsvMat, hsvLower, hsvUpper, rangeMat)
        }

        // erode + dilate remove small areas and fill gaps
        Imgproc.dilate(rangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 2)
        Imgproc.erode(denoisedRangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 6)
        Imgproc.dilate(denoisedRangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 4)

        // finds contours in the filtered mat
        Imgproc.findContours(denoisedRangeMat, goalContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(goalContourMat, goalContours, -1, Scalar(0.0, 255.0, 0.0), 6)

        goalContours.forEach { contour ->
            // contour must pass a minimum width check
            val rect = Imgproc.boundingRect(contour)
            if (rect.width < MIN_GOAL_WIDTH) return@forEach

            // convert contour into convex contour
            Imgproc.convexHull(contour, hullIndices)
            val contourArray = contour.toArray()
            contour.fromList(hullIndices.toArray().map { contourArray[it] })

            Imgproc.drawContours(goalContourMat, listOf(contour), -1, Scalar(255.0, 255.0, 0.0), 3)

            // fill the convex contour
            Imgproc.fillConvexPoly(convexGoalMat, contour, Scalar(255.0))
        }

        Core.copyTo(input, finalMat, convexGoalMat)

        Core.subtract(convexGoalMat, denoisedRangeMat, slotMat)

        Imgproc.erode(slotMat, denoisedSlotMat, wideElement, Point(-1.0, -1.0), 2)
        Imgproc.dilate(denoisedSlotMat, denoisedSlotMat, wideElement, Point(-1.0, -1.0), 2)

        // draw slot candidate contours
        Imgproc.findContours(denoisedSlotMat, slotContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(slotContourMat, slotContours, -1, Scalar(0.0, 255.0, 0.0), 6)


        slotContours.forEach { contour ->
            // contour must pass a width an ratio checks
            val rect = Imgproc.boundingRect(contour)
            val ratio = rect.width.toDouble() / rect.height.toDouble()
            if (rect.width < MIN_SLOT_WIDTH || rect.width > MAX_SLOT_WIDTH
                    || ratio < MIN_SLOT_RATIO || ratio > MAX_SLOT_RATIO) return@forEach

            // convert contour into approximated contour
            contour2f.fromArray(*contour.toArray())
            val arcLength = Imgproc.arcLength(contour2f, true)
            Imgproc.approxPolyDP(contour2f, approx2f, arcLength * 0.02, true)
            approx.fromArray(*approx2f.toArray())

            Imgproc.drawContours(slotContourMat, listOf(approx), -1, Scalar(255.0, 255.0, 0.0), 3)

            // approximation should be a quadrilateral
            if (approx.toArray().size != 4) return@forEach

            // fill the final goal contour
            Imgproc.drawContours(selectedSlotMat, listOf(contour), -1, Scalar(255.0), -1)

            Imgproc.dilate(selectedSlotMat, dilatedSelectedSlotMat, normalElement, Point(-1.0, -1.0), 3)
            Core.bitwise_and(dilatedSelectedSlotMat, denoisedRangeMat, selectedSlotEdgeMat)

            Imgproc.dilate(selectedSlotEdgeMat, selectedSlotEdgeMat, normalElement, Point(-1.0, -1.0), 3)

            // find goal slot corners using the selected slot's edge as a mask
            Imgproc.goodFeaturesToTrack(cColorMat, corners, 4, 0.02, 6.0,
                    selectedSlotEdgeMat, 5, 5, false, 0.04)

            // must have four corners on the slot :P
            if (corners.rows() != 4) return@forEach

            // refine the detected corner coordinates
            corners2f.fromArray(*corners.toArray())
            Imgproc.cornerSubPix(
                    cColorMat, corners2f, Size(9.0, 9.0), Size(-1.0, -1.0),
                    TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 40, 0.001)
            )

            val points = corners2f.toArray()

            // sort in the order defined for the slot coordinates (first left to right, then vertical comparisons)
            Arrays.sort(points, Comparator.comparingDouble { a: Point -> a.x })

            if (points[1].y < points[0].y) {
                val temp = points[0]
                points[0] = points[1]
                points[1] = temp
            }

            if (points[3].y < points[2].y) {
                val temp = points[2]
                points[2] = points[3]
                points[3] = temp
            }

            corners2f.fromArray(*points)

            Calib3d.solvePnP(SLOT_POINTS, corners2f, camera, DISTORTION_PARAMS, rvec, tvec, false, Calib3d.SOLVEPNP_AP3P)

            println(tvec.dump())
            println(rvec.dump())

            val impts = MatOfPoint2f()

            Calib3d.projectPoints(
                    MatOfPoint3f(Point3(12.0, 0.0, 0.0), Point3(0.0, 12.0, 0.0),
                            Point3(0.0, 0.0, 12.0),
                            Point3(25.0, 9.0, 1.0), Point3(32.5, 9.0, 1.0), Point3(40.0, 9.0, 1.0)),
                    rvec,
                    tvec,
                    camera,
                    DISTORTION_PARAMS,
                    impts
            )

            Imgproc.line(finalMat, points[0], impts.toArray()[0], Scalar(255.0, 0.0, 0.0), 3)
            Imgproc.line(finalMat, points[0], impts.toArray()[1], Scalar(0.0, 255.0, 0.0), 3)
            Imgproc.line(finalMat, points[0], impts.toArray()[2], Scalar(0.0, 0.0, 255.0), 3)

            Imgproc.circle(finalMat, impts.toArray()[3], 5, Scalar(255.0, 255.0, 0.0), 1)
            Imgproc.circle(finalMat, impts.toArray()[4], 5, Scalar(255.0, 255.0, 0.0), 1)
            Imgproc.circle(finalMat, impts.toArray()[5], 5, Scalar(255.0, 255.0, 0.0), 1)

            points.forEach { Imgproc.circle(finalMat, it, 5, Scalar(0.0, 255.0, 0.0), 1) }
        }

        goalContours.clear()
        slotContours.clear()

        val debugMats = arrayOf(
                finalMat, filteredMat, /* yMat, crMat, cbMat, */ denoisedRangeMat, goalContourMat,
                convexGoalMat, denoisedSlotMat, slotContourMat, selectedSlotMat, selectedSlotEdgeMat
        )
        return debugMats[stageNum % debugMats.size]
    }
}