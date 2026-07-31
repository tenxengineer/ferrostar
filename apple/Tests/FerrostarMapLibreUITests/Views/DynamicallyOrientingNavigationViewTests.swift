import CoreLocation
import Foundation
import MapLibreSwiftUI
import SwiftUI
import TestSupport
import XCTest
@testable import FerrostarMapLibreUI

final class DynamicallyOrientingViewTests: XCTestCase {
    func testUzMapOptionsAreRetained() {
        let view = DynamicallyOrientingNavigationView(
            styleURL: URL(
                string: "https://demotiles.maplibre.org/style.json"
            )!,
            camera: .constant(.default()),
            navigationState: .pedestrianExample,
            isMuted: false,
            showZoom: false,
            onTapMute: {},
            onStyleLoaded: { _ in }
        )

        XCTAssertFalse(view.showZoom)
        XCTAssertNotNil(view.onStyleLoaded)
    }

    func testNavigationStartRecenterUsesNavigationCamera() {
        var camera = MapViewCamera.center(
            CLLocationCoordinate2D(latitude: 41, longitude: 69),
            zoom: 8
        )
        let navigationCamera = MapViewCamera.center(
            CLLocationCoordinate2D(latitude: 41.25, longitude: 69.25),
            zoom: 16,
            pitch: 45,
            direction: 180
        )
        let view = DynamicallyOrientingNavigationView(
            styleURL: URL(
                string: "https://demotiles.maplibre.org/style.json"
            )!,
            camera: Binding(
                get: { camera },
                set: { camera = $0 }
            ),
            navigationCamera: navigationCamera,
            navigationState: .pedestrianExample,
            isMuted: false,
            onTapMute: {}
        )

        view.recenterNavigationCamera()

        var expected = navigationCamera
        expected.lastReasonForChange = .programmatic
        XCTAssertEqual(camera, expected)
    }

    func testUzMapCompatibilityIsWiredIntoRenderedView() throws {
        let appleRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let sourceURL = appleRoot.appendingPathComponent(
            "Sources/FerrostarMapLibreUI/Views/"
                + "DynamicallyOrientingNavigationView.swift"
        )
        let source = try String(contentsOf: sourceURL, encoding: .utf8)

        func occurrences(of needle: String) -> Int {
            source.components(separatedBy: needle).count - 1
        }

        func occurrencesIgnoringLeadingWhitespace(of needle: String) -> Int {
            func normalizeLeadingWhitespace(_ value: String) -> String {
                value
                    .split(separator: "\n", omittingEmptySubsequences: false)
                    .map { line in
                        line.drop { character in
                            character == " " || character == "\t"
                        }
                    }
                    .joined(separator: "\n")
            }

            return normalizeLeadingWhitespace(source)
                .components(
                    separatedBy: normalizeLeadingWhitespace(needle)
                )
                .count - 1
        }

        XCTAssertEqual(occurrences(of: "onStyleLoaded?(style)"), 1)
        XCTAssertEqual(
            occurrences(of: "showZoom: showZoom && isNavigating"),
            2
        )
        XCTAssertEqual(
            occurrencesIgnoringLeadingWhitespace(
                of: """
                .onChange(of: isNavigating) { navigating in
                    guard navigating else { return }
                    recenterNavigationCamera()
                }
                """
            ),
            1
        )
    }

    // TODO: This needs a fixed reference date for now. See TripProgressViewTests.
    //       The reason we haven't solved this is, it needs to be propagated through
    //       a much larger stack of views in this case.
//    func testDefault() {
//        assertView {
//            DynamicallyOrientingNavigationView(
//                styleURL: URL(string: "https://demotiles.maplibre.org/style.json")!,
//                camera: .constant(.automotiveNavigation()),
//                navigationState: .pedestrianExample,
//                isMuted: false,
//                onTapMute: {}
//            )
//            .navigationFormatterCollection(TestingFormatterCollection())
//        }
//    }

    func testCustomized() {
        assertView {
            DynamicallyOrientingNavigationView(
                styleURL: URL(string: "https://demotiles.maplibre.org/style.json")!,
                camera: .constant(.automotiveNavigation()),
                navigationState: .pedestrianExample,
                isMuted: false,
                onTapMute: {}
            )
            .navigationViewProgressView { state, _ in
                Text("Progress: \(state?.currentProgress?.distanceToNextManeuver ?? -1)")
                    .background(Color.blue)
                    .padding()
            }
            .navigationViewInstructionView { state, _, _ in
                Text("Instruction: \(state?.currentVisualInstruction?.primaryContent.text ?? "unknown")")
                    .background(Color.purple)
                    .padding()
            }
            .navigationViewCurrentRoadView { state in
                Text("Current Road: \(state?.currentRoadName ?? "unknown")")
                    .background(Color.yellow)
                    .padding()
            }
            .navigationFormatterCollection(TestingFormatterCollection())
        }
    }
}
