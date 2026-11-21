#include "guikit/guikit.hpp"
#include "guikit/cockpit.hpp"
#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
#include "guikit/backend/thor_canvas.hpp"
#endif
#include "keel/keel.h"
#include "keel/math.h"

#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
using TestCanvas = guikit::ThorCanvas;
#else
using TestCanvas = guikit::NullCanvas;
#endif

static keel::i32 g_failures = 0;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            keel::eprintln("[FAIL] {} ({}:{})", #cond, __FILE__, __LINE__);          \
            ++g_failures;                                                            \
        }                                                                            \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                        \
    do {                                                                             \
        if (keel::abs((a) - (b)) > (eps)) {                                           \
            keel::eprintln("[FAIL] {} ({}) != {} ({}) [{}:{}]",                      \
                           #a, (a), #b, (b), __FILE__, __LINE__);                    \
            ++g_failures;                                                            \
        }                                                                            \
    } while (0)

using namespace guikit;

void TestAttitudeViewWidgetWorkflow() {
    using namespace guikit;
    Context ui;
    TestCanvas canvas;
    const keel::u32 width = 900;
    const keel::u32 height = 650;
    keel::Array<keel::u32> pixels(width * height);
    canvas.SetTarget(pixels.data(), width, height, width);

    keel::f32 fovPitch = 50.0f;
    AttitudeDisplayMode mode = AttitudeDisplayMode::FullPFD;
    HorizonReferenceMode refMode = HorizonReferenceMode::WesternMovingHorizon;
    AttitudeColorScheme scheme = AttitudeColorScheme::GlassCockpit;
    keel::f32 roll = 28.0f;
    keel::f32 pitch = 12.5f;
    AttitudeViewResponse resp{};

    auto renderFrame = [&]() {
        ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
        if (auto col = ui.Column({
            .padding = Padding::All(12.0f),
            .width = SizeMode::Stretch,
            .height = SizeMode::Stretch
        })) {
            resp = guikit::cockpit::AttitudeView(ui, {
                .displayMode = mode,
                .referenceMode = refMode,
                .colorScheme = scheme,
                .flightDirectorStyle = FlightDirectorStyle::SingleCueVBar,
                .aircraftSymbol = AircraftSymbolStyle::GullWing,
                .rollDeg = roll,
                .pitchDeg = pitch,
                .headingDeg = 135.0f,
                .airspeedKnots = 265.0f,
                .machNumber = 0.76f,
                .altitudeFeet = 18400.0f,
                .verticalSpeedFpm = 1450.0f,
                .slipSkid = -0.25f,
                .gLoad = 1.35f,
                .radioAltitudeFeet = 1200.0f,
                .showFlightPathVector = true,
                .showIlsDiamonds = true,
                .locDeviationDots = 0.6f,
                .gsDeviationDots = -0.4f,
                .showMouseStickOverlay = true,
                .fieldOfViewPitchDeg = fovPitch,
                .fovPitchDegPtr = &fovPitch,
                .height = SizeMode::Stretch
            });
        }
        ui.EndFrame(canvas);
    };

    // Frame 1 & 2: Establish layout and verify ADI sphere geometry
    renderFrame();
    renderFrame();
    CHECK(resp.rect.w > 600.0f);
    CHECK(resp.rect.h > 400.0f);
    CHECK(resp.adiRect.w > 200.0f);
    CHECK(resp.adiRadius > 100.0f);
    CHECK_NEAR(resp.visibleFovPitchDeg, 50.0f, 0.01f);

    // Frame 3: Move mouse inside ADI sphere (right and up from center -> positive roll, positive pitch)
    const Vec2 hoverPt{resp.adiCenter.x + resp.adiRadius * 0.5f, resp.adiCenter.y - resp.adiRadius * 0.5f};
    ui.OnMouseMove(hoverPt);
    renderFrame();
    CHECK(resp.hovered);
    CHECK(resp.cursorInside);
    CHECK(resp.cursorInsideAdi);
    CHECK(resp.cursorNorm.x > 0.45f);
    CHECK(resp.cursorNorm.y < -0.45f);
    CHECK(resp.cursorRollDeg > 25.0f);
    CHECK(resp.cursorPitchDeg > 15.0f);

    // Frame 4: Mouse wheel zoom changes pitch FOV
    ui.OnMouseWheel(0.0f, 2.0f);
    renderFrame();
    CHECK(resp.changed);
    CHECK_NEAR(fovPitch, 42.0f, 0.01f);

    // Frame 5: Test CompactADI + Russian KPPI Reference Mode + Inverted Flight (roll = 175 deg, pitch = -45 deg)
    mode = AttitudeDisplayMode::CompactADI;
    refMode = HorizonReferenceMode::EasternMovingAircraft;
    scheme = AttitudeColorScheme::TacticalDark;
    roll = 175.0f;
    pitch = -45.0f;
    renderFrame();
    CHECK(resp.displayMode == AttitudeDisplayMode::CompactADI);

    // Frame 6: Test HudOverlay mode + HudGreen color scheme
    mode = AttitudeDisplayMode::HudOverlay;
    scheme = AttitudeColorScheme::HudGreen;
    roll = -62.0f;
    pitch = 38.0f;
    renderFrame();
    CHECK(resp.displayMode == AttitudeDisplayMode::HudOverlay);
}

void TestCarClusterViewWidgetWorkflow() {
    using namespace guikit;
    Context ui;
    TestCanvas canvas;
    const keel::u32 width = 960;
    const keel::u32 height = 560;
    keel::Array<keel::u32> pixels(width * height);
    canvas.SetTarget(pixels.data(), width, height, width);

    ClusterLayoutMode layoutMode = ClusterLayoutMode::DualDial;
    PowertrainMode powertrain = PowertrainMode::IceTurbo;
    ClusterDriveTheme driveTheme = ClusterDriveTheme::ComfortCyan;
    ClusterCenterMfd centerMfd = ClusterCenterMfd::AdasLaneRadar;
    SpeedUnit speedUnit = SpeedUnit::Kmh;
    keel::i32 gear = 4;
    CarClusterViewResponse resp{};

    auto renderFrame = [&]() {
        ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
        if (auto col = ui.Column({
            .padding = Padding::All(12.0f),
            .width = SizeMode::Stretch,
            .height = SizeMode::Stretch
        })) {
            resp = guikit::cockpit::CarClusterView(ui, {
                .layoutMode = layoutMode,
                .powertrain = powertrain,
                .driveTheme = driveTheme,
                .centerMfd = centerMfd,
                .speedUnit = speedUnit,
                .gearSelector = GearSelector::Sport,
                .speedKmh = 148.0f,
                .rpm = 5800.0f,
                .currentGear = gear,
                .evPowerKw = 185.0f,
                .throttlePct = 64.0f,
                .brakePct = 0.0f,
                .steeringAngleDeg = 14.0f,
                .boostBar = 1.15f,
                .fuelOrSocPct = 72.0f,
                .latG = 0.68f,
                .lonG = 0.42f,
                .showMouseDriveOverlay = true,
                .manualGearPtr = &gear,
                .centerMfdPtr = &centerMfd,
                .height = SizeMode::Stretch
            });
        }
        ui.EndFrame(canvas);
    };

    // Frame 1 & 2: Establish layout and verify DualDial sub-rectangles
    renderFrame();
    renderFrame();
    CHECK(resp.rect.w > 600.0f);
    CHECK(resp.rect.h > 300.0f);
    CHECK(resp.clusterRect.w > 500.0f);
    CHECK(resp.leftGaugeRect.w > 100.0f);
    CHECK(resp.centerMfdRect.w > 80.0f);
    CHECK(resp.rightGaugeRect.w > 100.0f);

    // Frame 3: Hover mouse in upper-right quadrant -> positive steering, positive throttle
    const Vec2 clusterCenter = resp.clusterRect.Center();
    ui.OnMouseMove({clusterCenter.x + resp.clusterRect.w * 0.25f, clusterCenter.y - resp.clusterRect.h * 0.25f});
    renderFrame();
    CHECK(resp.hovered);
    CHECK(resp.cursorInside);
    CHECK(resp.cursorSteerDeg > 15.0f);
    CHECK(resp.cursorThrottlePct > 40.0f);
    CHECK_NEAR(resp.cursorBrakePct, 0.0f, 0.01f);

    // Frame 4: Mouse wheel scroll upshifts manualGearPtr (4 -> 5)
    ui.OnMouseWheel(0.0f, 1.0f);
    renderFrame();
    CHECK(resp.changed);
    CHECK(gear == 5);

    // Frame 5: Click inside Center MFD cycles centerMfdPtr (AdasLaneRadar -> TpmsChassis)
    ui.OnMouseMove(resp.centerMfdRect.Center());
    ui.OnMouseDown(1);
    renderFrame();
    ui.OnMouseUp(1);
    renderFrame();
    CHECK(centerMfd == ClusterCenterMfd::TpmsChassis);

    // Frame 6: Test TrackCentral layout + ElectricDualMotor + GForceDynamics + Imperial mph
    layoutMode = ClusterLayoutMode::TrackCentral;
    powertrain = PowertrainMode::ElectricDualMotor;
    driveTheme = ClusterDriveTheme::SportCrimson;
    centerMfd = ClusterCenterMfd::GForceDynamics;
    speedUnit = SpeedUnit::Mph;
    renderFrame();
    CHECK(resp.layoutMode == ClusterLayoutMode::TrackCentral);

    // Frame 7: Test ModernWide layout + TrackGold theme
    layoutMode = ClusterLayoutMode::ModernWide;
    driveTheme = ClusterDriveTheme::TrackGold;
    renderFrame();
    CHECK(resp.layoutMode == ClusterLayoutMode::ModernWide);
}

int main() {
    keel::println("[RUN] Testing GuiKit Cockpit module...");
    TestAttitudeViewWidgetWorkflow();
    TestCarClusterViewWidgetWorkflow();
    if (g_failures == 0) {
        keel::println("[PASS] All GuiKit Cockpit tests passed successfully!");
        return 0;
    }
    keel::eprintln("[FAIL] Total failures: {}", g_failures);
    return 1;
}
