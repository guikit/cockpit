#pragma once

#include "guikit/core/control.hpp"
#include "guikit/core/response.hpp"
#include "guikit/style/theme.hpp"
#include "guikit/style/tokens.hpp"
#include "guikit/core/types.hpp"
#include "guikit/graphics/icon.hpp"
#include "keel/core.h"
#include "keel/optional.h"

namespace guikit {

class Context;
class ICanvas;

/// Display layout mode for the AttitudeView (Artificial Horizon / Primary Flight Display) widget.
enum class AttitudeDisplayMode : keel::u8 {
    FullPFD = 0,        ///< Complete Primary Flight Display: ADI horizon + IAS speed tape + Altitude & VSI tapes + Heading tape + FMA bar + ILS diamonds
    CompactADI,         ///< Standalone Attitude Director Indicator (artificial horizon without side tapes, with digital corner readouts)
    HudOverlay          ///< Transparent Head-Up Display (HUD) collimator vector symbology without opaque sky/ground fill
};

/// Viewport geometry and clipping mode for the artificial horizon sphere.
enum class AttitudeViewportMode : keel::u8 {
    Auto = 0,           ///< Automatic shape based on displayMode (RoundedDome in FullPFD/HudOverlay, Circular in CompactADI)
    FullBleed,          ///< Edge-to-edge full-screen horizon (Garmin G1000 / G3000 style) with tapes overlaid on top
    RoundedDome,        ///< Classic central rounded-square ADI dome (Airbus / Boeing PFD style)
    Circular            ///< Classic circular ADI sphere bezel
};

/// Coordinate reference frame for the artificial horizon and aircraft symbol.
enum class HorizonReferenceMode : keel::u8 {
    WesternMovingHorizon = 0, ///< Western ("Inside-Out" / View from cockpit): Fixed aircraft symbol, horizon rolls and pitches
    EasternMovingAircraft     ///< Russian KPPI / AGD ("Outside-In" / View from ground): Horizon pitches vertically while aircraft silhouette rolls
};

/// Color palette and avionics visual styling for the AttitudeView instrument.
enum class AttitudeColorScheme : keel::u8 {
    GlassCockpit = 0,   ///< Classic avionics sky blue & earth brown (Garmin G1000 / Airbus / Boeing PFD style)
    HudGreen,           ///< High-contrast monochrome phosphor green HUD symbology
    TacticalDark        ///< Modern dark synth-vision telemetry theme matching the GuiKit dark palette
};

/// Flight Director (FD) command cue symbology mode.
enum class FlightDirectorStyle : keel::u8 {
    Off = 0,            ///< Flight Director hidden
    SingleCueVBar,      ///< Single-cue magenta V-Bar wedge command indicator
    DualCueCrossbars    ///< Dual-cue magenta pitch & roll crossbar needles
};

/// Central fixed/rolling aircraft reference symbol style.
enum class AircraftSymbolStyle : keel::u8 {
    GullWing = 0,       ///< Classic yellow/amber twin L-wings with center bore-sight square
    ClassicW,           ///< W-shaped wing reference symbol with center apex
    RussianSilhouette   ///< KPPI winged silhouette with vertical stabilizer fin
};

/// Configuration options for the autonomous AttitudeView (Artificial Horizon / PFD) widget.
struct AttitudeViewOptions {
    // --- Display Mode, Viewport Shape, Reference Frame & Visual Theme ---
    AttitudeDisplayMode displayMode{AttitudeDisplayMode::FullPFD};
    AttitudeViewportMode viewportMode{AttitudeViewportMode::Auto};
    HorizonReferenceMode referenceMode{HorizonReferenceMode::WesternMovingHorizon};
    AttitudeColorScheme colorScheme{AttitudeColorScheme::GlassCockpit};
    FlightDirectorStyle flightDirectorStyle{FlightDirectorStyle::SingleCueVBar};
    AircraftSymbolStyle aircraftSymbol{AircraftSymbolStyle::GullWing};

    // --- Primary Attitude & Air Data Telemetry ---
    keel::f32 rollDeg{0.0f};                ///< Bank angle in degrees (-180 .. +180, positive = right wing down)
    keel::f32 pitchDeg{0.0f};               ///< Pitch angle in degrees (-90 .. +90, positive = nose up)
    keel::f32 headingDeg{0.0f};             ///< Magnetic/True heading in degrees [0 .. 360)
    keel::f32 airspeedKnots{250.0f};        ///< Indicated Airspeed (IAS) in knots
    keel::f32 machNumber{0.72f};            ///< Mach number readout below speed tape
    keel::f32 altitudeFeet{12500.0f};       ///< Barometric altitude in feet
    keel::f32 verticalSpeedFpm{0.0f};       ///< Vertical speed in feet per minute (-6000 .. +6000)
    keel::f32 slipSkid{0.0f};               ///< Lateral inclinometer slip/skid ball offset (-1.0 .. +1.0)
    keel::f32 gLoad{1.0f};                  ///< Normal load factor Nz in G
    keel::f32 radioAltitudeFeet{2500.0f};   ///< Radio altimeter AGL height in feet (displayed when < 2500 ft)

    // --- Flight Path Vector (FPV), Flight Director (FD) & Autopilot Bugs ---
    bool showFlightPathVector{true};    ///< Render Flight Path Vector ("bird" / velocity vector reticle)
    keel::f32 aoaDeg{2.2f};                 ///< Angle of Attack (alpha, shifts FPV vertically relative to pitch)
    keel::f32 sideslipDeg{0.0f};            ///< Sideslip angle (beta, shifts FPV horizontally)
    keel::f32 fdPitchDeg{2.5f};             ///< Flight Director commanded pitch in degrees
    keel::f32 fdRollDeg{0.0f};              ///< Flight Director commanded roll in degrees
    keel::f32 targetAirspeedBug{250.0f};    ///< Autopilot selected airspeed bug in knots
    keel::f32 targetAltitudeBug{12500.0f};  ///< Autopilot selected altitude bug in feet
    keel::f32 targetHeadingBug{0.0f};       ///< Autopilot selected heading bug in degrees
    keel::f32 baroSettingInHg{29.92f};      ///< Barometric altimeter setting in inHg (e.g. 29.92)
    bool baroStd{false};                ///< Display "STD" barometric reference instead of inHg

    // --- ILS Approach Localizer & Glideslope Deviation ---
    bool showIlsDiamonds{true};         ///< Render ILS Localizer (bottom) and Glideslope (right) deviation scales
    keel::f32 locDeviationDots{0.0f};       ///< Localizer lateral deviation in dots (-2.5 .. +2.5)
    keel::f32 gsDeviationDots{0.0f};        ///< Glideslope vertical deviation in dots (-2.5 .. +2.5)

    // --- Symbology & Sub-Instrument Visibility Toggles ---
    bool showPitchLadder{true};         ///< Render 2.5/5/10 deg pitch ladder, zenith/nadir reticles, and chevrons
    bool showBankScale{true};           ///< Render upper bank arc (10/20/30/45/60 deg), roll pointer, and slip/skid ball
    bool showSpeedAndAltTapes{true};    ///< Render left Airspeed and right Altitude + VSI tapes in FullPFD mode
    bool showHeadingTape{true};         ///< Render magnetic compass heading tape
    bool showFmaBar{true};              ///< Render top Flight Mode Annunciator (FMA) status strip
    bool showMouseStickOverlay{false};  ///< Render interactive virtual-stick / mouse control reticle inside the ADI
    keel::f32 fieldOfViewPitchDeg{50.0f};   ///< Vertical pitch span visible across the ADI diameter (30 .. 90 deg)

    // --- Optional Two-Way Bindings ---
    keel::f32* fovPitchDegPtr{nullptr};     ///< Optional external two-way binding for mouse-wheel pitch FOV zoom

    // --- Sizing & Styling ---
    AxisSize width{};                     ///< Horizontal sizing policy (`Auto` -> `Stretch`, `Fit`, or `Fixed(dp)`)
    AxisSize height{SizeMode::Stretch};   ///< Vertical sizing policy (`Stretch` default, `Fit`, or `Fixed(dp)`)
    StateColor background{};
    Border border{};
    UiSize size{UiSize::Md};
    bool disabled{false};
    UiText tooltip{};
};

/// Rich telemetry and cursor interaction state returned by `ui.AttitudeView(...)`.
struct AttitudeViewResponse : public Response {
    AttitudeDisplayMode displayMode{AttitudeDisplayMode::FullPFD};
    AttitudeViewportMode viewportMode{AttitudeViewportMode::RoundedDome};
    Rect horizonRect{};                 ///< Resolved screen rectangle of the sky/ground horizon fill area
    Rect adiRect{};                     ///< Resolved screen rectangle of the central artificial horizon symbology zone
    Vec2 adiCenter{};                   ///< Center point of the artificial horizon in screen coordinates
    keel::f32 adiRadius{0.0f};              ///< Radius of the artificial horizon sphere in pixels
    keel::f32 visibleFovPitchDeg{50.0f};    ///< Effective vertical pitch field-of-view in degrees

    bool cursorInside{false};           ///< True when mouse cursor is inside the widget control area
    bool cursorInsideAdi{false};        ///< True when mouse cursor is inside the ADI horizon region
    Vec2 cursorNorm{0.0f, 0.0f};        ///< Normalized cursor offset from ADI center in [-1.0, +1.0] (X = roll, Y = pitch)
    keel::f32 cursorRollDeg{0.0f};          ///< Roll angle mapped from cursor X position (-65 .. +65 deg)
    keel::f32 cursorPitchDeg{0.0f};         ///< Pitch angle mapped from cursor Y position (-40 .. +40 deg, up = nose up)
};

/// Autonomous Avionics Attitude Director Indicator (ADI) & Primary Flight Display (PFD) widget.
class AttitudeView {
public:
    static AttitudeViewResponse Run(
        Context& ctx,
        const AttitudeViewOptions& opts = {},
        keel::SourceLocation loc = keel::SourceLocation::current()
    );

    [[nodiscard]] static Vec2 Measure(
        const AttitudeViewOptions& opts,
        UiSize size,
        ICanvas* canvas = nullptr,
        const UiTheme& theme = {},
        keel::f32 availableWidth = 0.0f
    );
};

} // namespace guikit
