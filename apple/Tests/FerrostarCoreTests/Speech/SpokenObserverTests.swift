import AVFoundation
import Combine
import FerrostarCoreFFI
import XCTest
@testable import FerrostarCore

final class MockSpeechSynthesizer: SpeechSynthesizer {
    var isSpeaking: Bool = false

    var onSpeak: ((AVSpeechUtterance) -> Void)?
    func speak(_ utterance: AVSpeechUtterance) {
        onSpeak?(utterance)
    }

    var onStopSpeaking: ((AVSpeechBoundary) -> Void)?
    func stopSpeaking(at boundary: AVSpeechBoundary) -> Bool {
        onStopSpeaking?(boundary)
        return true
    }
}

final class SpokenObserverTests: XCTestCase {
    var cancellables = Set<AnyCancellable>()

    func test_mute() {
        let mockSpeechSynthesizer = MockSpeechSynthesizer()
        let spokenObserver = SpokenInstructionObserver(synthesizer: mockSpeechSynthesizer, isMuted: false)

        let muteExp = expectation(description: "isMuted is set to true")
        spokenObserver.$isMuted
            .sink { newIsMuted in
                guard newIsMuted else {
                    return
                }
                muteExp.fulfill()
            }
            .store(in: &cancellables)

        let exp = expectation(description: "stop speaking is called")
        mockSpeechSynthesizer.onStopSpeaking = { boundary in
            XCTAssertEqual(boundary, .immediate)
            exp.fulfill()
        }

        spokenObserver.toggleMute()

        wait(for: [muteExp, exp], timeout: 10)
    }

    func test_speakWhileMuted() {
        let mockSpeechSynthesizer = MockSpeechSynthesizer()
        let spokenObserver = SpokenInstructionObserver(synthesizer: mockSpeechSynthesizer, isMuted: true)

        mockSpeechSynthesizer.onSpeak = { _ in
            XCTFail("Speak should never be called when isMuted is true")
        }

        let exp = expectation(description: "task complete")
        Task {
            spokenObserver.spokenInstructionTriggered(.init(
                text: "Speak",
                ssml: "Speak",
                triggerDistanceBeforeManeuver: 1.0,
                utteranceId: .init()
            ))
            try await Task.sleep(nanoseconds: 1_000_000_000)
            exp.fulfill()
        }

        wait(for: [exp], timeout: 10)
    }

    func test_speakWhileUnmuted() {
        let mockSpeechSynthesizer = MockSpeechSynthesizer()
        let spokenObserver = SpokenInstructionObserver(synthesizer: mockSpeechSynthesizer, isMuted: false)

        let exp = expectation(description: "speak is called with expected text")
        mockSpeechSynthesizer.onSpeak = { utterance in
            XCTAssertEqual(utterance.speechString, "Speak")
            exp.fulfill()
        }

        let taskExp = expectation(description: "task complete")
        Task {
            spokenObserver.spokenInstructionTriggered(.init(
                text: "Speak",
                ssml: "Speak",
                triggerDistanceBeforeManeuver: 1.0,
                utteranceId: .init()
            ))
            try await Task.sleep(nanoseconds: 1_000_000_000)
            taskExp.fulfill()
        }

        wait(for: [exp, taskExp], timeout: 10)
    }

    @MainActor func test_structuredInstructionTransformReceivesTriggeringTripState() {
        let mockSpeechSynthesizer = MockSpeechSynthesizer()
        let spokenObserver = SpokenInstructionObserver(
            synthesizer: mockSpeechSynthesizer,
            isMuted: false,
            instructionTransform: { instruction, tripState in
                XCTAssertTrue(Thread.isMainThread)
                guard case .idle = tripState else {
                    XCTFail("Expected the triggering trip state")
                    return instruction
                }
                return .init(
                    text: "Transformed",
                    ssml: nil,
                    triggerDistanceBeforeManeuver: instruction.triggerDistanceBeforeManeuver,
                    utteranceId: instruction.utteranceId
                )
            }
        )

        let exp = expectation(description: "transformed instruction is spoken")
        mockSpeechSynthesizer.onSpeak = { utterance in
            XCTAssertEqual(utterance.speechString, "Transformed")
            exp.fulfill()
        }

        spokenObserver.spokenInstructionTriggered(
            .init(
                text: "Original",
                ssml: nil,
                triggerDistanceBeforeManeuver: 100,
                utteranceId: .init()
            ),
            tripState: .idle(userLocation: nil)
        )

        wait(for: [exp], timeout: 10)
    }
}
