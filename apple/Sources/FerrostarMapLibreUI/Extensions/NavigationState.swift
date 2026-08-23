import FerrostarCore
import MapLibre
import MapLibreSwiftUI

public extension NavigationState {
    /// The MapViewCamera representing the route polyline showcase.
    ///
    /// UzMap P5 (vendor `guidance_camera` OverviewMode): the overview is a north-up 2D fit of the
    /// REMAINING route — not the full polyline including the already-driven tail. Mirrors the
    /// Android `fullRouteGeometryIndex` + `drop(index)` slice in `NavigationScene.kt`.
    var routeOverviewCamera: MapViewCamera? {
        // Remaining-route slice (see the doc comment above): the full-route index of the current
        // position is `full.count − remainingDeduped + stepLocal`, where remainingSteps overlap at
        // shared endpoints (minus one per joint). Type stays inferred — the FFI module is not a
        // direct dependency of this target.
        let full = routeGeometry
        var startIndex = 0
        if case let .navigating(currentStepGeometryIndex, _, _, remainingSteps, _, _, _, _, _, _, _, _) = tripState,
           let stepLocal = currentStepGeometryIndex,
           !remainingSteps.isEmpty, !full.isEmpty
        {
            let remainingDeduped = remainingSteps.reduce(0) { $0 + $1.geometry.count } - (remainingSteps.count - 1)
            startIndex = min(max(full.count - remainingDeduped + Int(stepLocal), 0), max(full.count - 1, 0))
        }
        let remaining = full.suffix(from: startIndex)
        guard let firstCoordinate = remaining.first else {
            return nil
        }

        let initial = MLNCoordinateBounds(
            sw: firstCoordinate.clLocationCoordinate2D,
            ne: firstCoordinate.clLocationCoordinate2D
        )
        let bounds = remaining.reduce(initial) { acc, coord in
            MLNCoordinateBounds(
                sw: CLLocationCoordinate2D(latitude: min(acc.sw.latitude, coord.lat), longitude: min(
                    acc.sw.longitude,
                    coord.lng
                )),
                ne: CLLocationCoordinate2D(
                    latitude: max(acc.ne.latitude, coord.lat),
                    longitude: max(acc.ne.longitude, coord.lng)
                )
            )
        }

        return MapViewCamera.boundingBox(bounds, edgePadding: UIEdgeInsets(top: 20, left: 100, bottom: 20, right: 100))
    }
}
