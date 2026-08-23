import CoreLocation
import FerrostarCoreFFI
import Foundation

/// A Swift wrapper around `UniFFI.NavigationControllerConfig`.
public struct SwiftNavigationControllerConfig {
    public init(waypointAdvance: WaypointAdvanceMode,
                stepAdvanceCondition: StepAdvanceCondition,
                arrivalStepAdvanceCondition: StepAdvanceCondition,
                routeDeviationTracking: SwiftRouteDeviationTracking,
                snappedLocationCourseFiltering: CourseFiltering,
                uzmatch: UzmatchConfig = UzmatchConfig(
                    enabled: false,
                    standingSpeedThresholdMps: 0.5,
                    standingDetectionPeriodMs: 7000,
                    standingSignalExpiryMs: 5000,
                    snapPositionStddevM: 8.0,
                    snapHeadingStddevDeg: 6.0,
                    snapHeadingMinSpeedMps: 4.0,
                    fineAccuracyThresholdM: 25.0
                ))
    {
        ffiValue = FerrostarCoreFFI.NavigationControllerConfig(
            waypointAdvance: waypointAdvance,
            stepAdvanceCondition: stepAdvanceCondition,
            arrivalStepAdvanceCondition: arrivalStepAdvanceCondition,
            routeDeviationTracking: routeDeviationTracking.ffiValue,
            snappedLocationCourseFiltering: snappedLocationCourseFiltering,
            uzmatch: uzmatch
        )
    }

    var ffiValue: FerrostarCoreFFI.NavigationControllerConfig
}
