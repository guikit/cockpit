#include "guikit/cockpit/attitude_view.hpp"
#include "guikit/backend/i_canvas.hpp"
#include "guikit/context.hpp"
#include "guikit/core/control.hpp"
#include "guikit/core/node.hpp"
#include "keel/core.h"
#include "keel/format.h"
#include "keel/math.h"
#include "keel/ref.h"
#include "keel/small_array.h"
#include "keel/static_array.h"

namespace guikit {

namespace {

struct AttitudeViewState {
    keel::f32 fovPitchDeg{50.0f};
    bool initialized{false};
};

struct AttitudeViewPayload {
    AttitudeViewOptions opts{};
    AttitudeViewResponse resp{};
};

struct AttitudePalette {
    Color bgConsole{10, 14, 24, 255};
    Color bezelRing{30, 41, 59, 255};
    Color bezelHighlight{100, 116, 139, 140};
    Color skyFill{4, 118, 208, 255};
    Color skyGlow{68, 178, 250, 85};
    Color groundFill{122, 66, 24, 255};
    Color groundGlow{168, 98, 42, 75};
    Color horizonLine{255, 255, 255, 255};
    Color pitchLadder{248, 250, 252, 242};
    Color bankScale{248, 250, 252, 235};
    Color chevronAlert{244, 63, 94, 245};
    Color aircraftFill{250, 204, 21, 255};
    Color aircraftStroke{10, 14, 24, 245};
    Color flightDirector{236, 72, 246, 255};
    Color fpvColor{52, 245, 118, 250};
    Color tapeBg{13, 19, 33, 215};
    Color tapeBorder{148, 163, 184, 150};
    Color tapeTick{241, 245, 249, 230};
    Color bugCyan{34, 211, 238, 255};
    Color ilsMagenta{236, 72, 246, 255};
    Color fmaGreen{74, 222, 128, 255};
    Color fmaCyan{56, 189, 248, 255};
    Color fmaWhite{248, 250, 252, 250};
    Color amberWarn{251, 191, 36, 255};
    Color redLimit{239, 68, 68, 240};
    Color greenArc{34, 197, 94, 210};
};

[[nodiscard]] AttitudePalette BuildPalette(AttitudeColorScheme scheme, AttitudeDisplayMode mode) noexcept {
    AttitudePalette p{};
    if (scheme == AttitudeColorScheme::HudGreen || mode == AttitudeDisplayMode::HudOverlay) {
        const bool isHudMode = (mode == AttitudeDisplayMode::HudOverlay);
        p.bgConsole = isHudMode ? Color(5, 14, 10, 235) : Color(6, 16, 11, 255);
        p.bezelRing = Color(16, 48, 28, 220);
        p.bezelHighlight = Color(52, 245, 118, 110);
        p.skyFill = isHudMode ? Color(0, 0, 0, 0) : Color(8, 30, 18, 255);
        p.skyGlow = isHudMode ? Color(52, 245, 118, 18) : Color(52, 245, 118, 30);
        p.groundFill = isHudMode ? Color(0, 0, 0, 0) : Color(14, 42, 22, 255);
        p.groundGlow = isHudMode ? Color(52, 245, 118, 12) : Color(52, 245, 118, 22);
        const Color hudMain{52, 245, 118, 245};
        const Color hudBright{167, 255, 198, 255};
        p.horizonLine = hudBright;
        p.pitchLadder = hudMain;
        p.bankScale = hudMain;
        p.chevronAlert = Color(250, 204, 21, 245);
        p.aircraftFill = hudBright;
        p.aircraftStroke = Color(4, 16, 8, 220);
        p.flightDirector = hudBright;
        p.fpvColor = hudBright;
        p.tapeBg = Color(4, 18, 10, 175);
        p.tapeBorder = Color(52, 245, 118, 140);
        p.tapeTick = hudMain;
        p.bugCyan = hudBright;
        p.ilsMagenta = hudBright;
        p.fmaGreen = hudBright;
        p.fmaCyan = hudMain;
        p.fmaWhite = hudMain;
        p.amberWarn = Color(250, 204, 21, 245);
        p.redLimit = Color(244, 63, 94, 235);
        p.greenArc = hudMain;
    } else if (scheme == AttitudeColorScheme::TacticalDark) {
        p.bgConsole = Color(8, 11, 20, 255);
        p.bezelRing = Color(22, 30, 48, 255);
        p.bezelHighlight = Color(56, 189, 248, 120);
        p.skyFill = Color(15, 38, 82, 255);
        p.skyGlow = Color(56, 189, 248, 65);
        p.groundFill = Color(52, 36, 26, 255);
        p.groundGlow = Color(180, 110, 52, 55);
        p.horizonLine = Color(56, 189, 248, 250);
    }
    return p;
}

[[nodiscard]] inline keel::f32 NormalizeHeadingDeg(keel::f32 deg) noexcept {
    keel::f32 r = keel::fmod(deg, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}

[[nodiscard]] inline keel::f32 SignedAngleDiffDeg(keel::f32 a, keel::f32 b) noexcept {
    return keel::fmod(a - b + 540.0f, 360.0f) - 180.0f;
}

/// Clips a convex polygon against the half-plane `(P - planeOrigin) . planeNormal <= 0`.
[[nodiscard]] keel::SmallArray<Vec2, 64> ClipPolygonHalfPlane(
    keel::ArrayRef<Vec2> poly,
    Vec2 planeOrigin,
    Vec2 planeNormal
) {
    keel::SmallArray<Vec2, 64> out;
    if (poly.empty()) return out;
    out.reserve(poly.size() + 2);

    auto evalDist = [&](Vec2 pt) noexcept -> keel::f32 {
        return (pt.x - planeOrigin.x) * planeNormal.x + (pt.y - planeOrigin.y) * planeNormal.y;
    };

    Vec2 prev = poly.back();
    keel::f32 prevDist = evalDist(prev);

    for (const Vec2 curr : poly) {
        const keel::f32 currDist = evalDist(curr);
        const bool prevInside = (prevDist <= 0.0f);
        const bool currInside = (currDist <= 0.0f);

        if (prevInside != currInside) {
            const keel::f32 denom = prevDist - currDist;
            const keel::f32 t = (keel::abs(denom) > 1e-6f) ? (prevDist / denom) : 0.5f;
            out.push_back(Vec2{
                prev.x + t * (curr.x - prev.x),
                prev.y + t * (curr.y - prev.y)
            });
        }
        if (currInside) {
            out.push_back(curr);
        }
        prev = curr;
        prevDist = currDist;
    }
    return out;
}

void DrawConvexPolygonFilled(ICanvas& canvas, keel::ArrayRef<Vec2> poly, Color fill) noexcept {
    if (poly.size() < 3 || fill.a == 0) return;
    Vec2 center{0.0f, 0.0f};
    for (const Vec2& p : poly) {
        center.x += p.x;
        center.y += p.y;
    }
    const keel::f32 invN = 1.0f / keel::as<keel::f32>(poly.size());
    center.x *= invN;
    center.y *= invN;

    for (keel::usize i = 0; i < poly.size(); ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % poly.size()];
        canvas.DrawTriangle(center, a, b, fill);
    }
}

/// Builds a full-bleed, rounded-dome, or circular convex polygon representing the ADI horizon boundary.
[[nodiscard]] keel::SmallArray<Vec2, 64> BuildAdiBoundaryPolygon(
    Rect horizonRect,
    AttitudeViewportMode viewportMode,
    keel::f32 outerCornerRadius = 0.0f
) {
    keel::SmallArray<Vec2, 64> pts;
    const Vec2 c = horizonRect.Center();
    if (viewportMode == AttitudeViewportMode::Circular) {
        constexpr keel::i32 kSteps = 48;
        pts.reserve(kSteps);
        const keel::f32 r = keel::min(horizonRect.w, horizonRect.h) * 0.5f;
        for (keel::i32 i = 0; i < kSteps; ++i) {
            const keel::f32 a = (keel::as<keel::f32>(i) / kSteps) * keel::kTauF;
            pts.push_back({c.x + keel::cos(a) * r, c.y + keel::sin(a) * r});
        }
        return pts;
    }

    const keel::f32 maxR = keel::min(horizonRect.w, horizonRect.h) * 0.5f;
    const keel::f32 cornerR = (viewportMode == AttitudeViewportMode::FullBleed)
        ? keel::clamp(outerCornerRadius, 0.0f, maxR)
        : (maxR * 0.44f);

    if (cornerR <= 0.5f) {
        pts.reserve(4);
        pts.push_back({horizonRect.x, horizonRect.y});
        pts.push_back({horizonRect.Right(), horizonRect.y});
        pts.push_back({horizonRect.Right(), horizonRect.Bottom()});
        pts.push_back({horizonRect.x, horizonRect.Bottom()});
        return pts;
    }

    const keel::f32 x0 = horizonRect.x + cornerR;
    const keel::f32 x1 = horizonRect.Right() - cornerR;
    const keel::f32 y0 = horizonRect.y + cornerR;
    const keel::f32 y1 = horizonRect.Bottom() - cornerR;
    constexpr keel::i32 kArcSteps = 8;
    pts.reserve(kArcSteps * 4 + 4);

    const keel::StaticArray<Vec2, 4> centers = {
        Vec2{x1, y0}, // Top-Right
        Vec2{x1, y1}, // Bottom-Right
        Vec2{x0, y1}, // Bottom-Left
        Vec2{x0, y0}  // Top-Left
    };
    const keel::StaticArray<keel::f32, 4> startAngles = {-keel::kHalfPiF, 0.0f, keel::kHalfPiF, keel::kPiF};

    for (keel::usize corner = 0; corner < 4; ++corner) {
        for (keel::i32 s = 0; s <= kArcSteps; ++s) {
            const keel::f32 a = startAngles[corner] + (keel::as<keel::f32>(s) / kArcSteps) * keel::kHalfPiF;
            pts.push_back({
                centers[corner].x + keel::cos(a) * cornerR,
                centers[corner].y + keel::sin(a) * cornerR
            });
        }
    }
    return pts;
}

/// Clips a 2D line segment (p0, p1) to a circle (center, radius). Returns true if any part is inside.
[[nodiscard]] bool ClipSegmentToCircle(Vec2& p0, Vec2& p1, Vec2 center, keel::f32 radius) noexcept {
    const Vec2 d{p1.x - p0.x, p1.y - p0.y};
    const Vec2 f{p0.x - center.x, p0.y - center.y};
    const keel::f32 a = d.x * d.x + d.y * d.y;
    if (a < 1e-6f) {
        return (f.x * f.x + f.y * f.y) <= radius * radius;
    }
    const keel::f32 b = 2.0f * (f.x * d.x + f.y * d.y);
    const keel::f32 c = (f.x * f.x + f.y * f.y) - radius * radius;
    keel::f32 disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return false;
    disc = keel::sqrt(disc);
    const keel::f32 t0 = (-b - disc) / (2.0f * a);
    const keel::f32 t1 = (-b + disc) / (2.0f * a);
    if (t1 < 0.0f || t0 > 1.0f) return false;
    const keel::f32 cl0 = keel::clamp(t0, 0.0f, 1.0f);
    const keel::f32 cl1 = keel::clamp(t1, 0.0f, 1.0f);
    if (cl1 <= cl0) return false;
    const Vec2 orig0 = p0;
    p0 = {orig0.x + cl0 * d.x, orig0.y + cl0 * d.y};
    p1 = {orig0.x + cl1 * d.x, orig0.y + cl1 * d.y};
    return true;
}

void DrawCircleOutline(ICanvas& canvas, Vec2 center, keel::f32 radius, Color stroke, keel::f32 strokeWidth = 1.5f, keel::i32 steps = 36) noexcept {
    if (radius <= 0.5f || stroke.a == 0) return;
    Vec2 prev{center.x + radius, center.y};
    for (keel::i32 i = 1; i <= steps; ++i) {
        const keel::f32 a = (keel::as<keel::f32>(i) / steps) * keel::kTauF;
        const Vec2 curr{center.x + keel::cos(a) * radius, center.y + keel::sin(a) * radius};
        canvas.DrawLine(prev, curr, stroke, strokeWidth);
        prev = curr;
    }
}

void DrawFilledCircle(ICanvas& canvas, Vec2 center, keel::f32 radius, Color fill, Color stroke = Color::Transparent(), keel::f32 strokeWidth = 0.0f, keel::i32 steps = 28) noexcept {
    if (radius <= 0.5f) return;
    if (fill.a > 0) {
        Vec2 prev{center.x + radius, center.y};
        for (keel::i32 i = 1; i <= steps; ++i) {
            const keel::f32 a = (keel::as<keel::f32>(i) / steps) * keel::kTauF;
            const Vec2 curr{center.x + keel::cos(a) * radius, center.y + keel::sin(a) * radius};
            canvas.DrawTriangle(center, prev, curr, fill);
            prev = curr;
        }
    }
    if (stroke.a > 0 && strokeWidth > 0.0f) {
        DrawCircleOutline(canvas, center, radius, stroke, strokeWidth, steps);
    }
}

void DrawQuadFilled(ICanvas& canvas, Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, Color fill, Color stroke = Color::Transparent(), keel::f32 strokeW = 1.0f) noexcept {
    if (fill.a > 0) {
        canvas.DrawTriangle(p0, p1, p2, fill);
        canvas.DrawTriangle(p0, p2, p3, fill);
    }
    if (stroke.a > 0 && strokeW > 0.0f) {
        canvas.DrawLine(p0, p1, stroke, strokeW);
        canvas.DrawLine(p1, p2, stroke, strokeW);
        canvas.DrawLine(p2, p3, stroke, strokeW);
        canvas.DrawLine(p3, p0, stroke, strokeW);
    }
}

/// Draws the Pitch Ladder (2.5 / 5 / 10 deg bars, negative dashed bars, zenith/nadir stars, recovery chevrons).
void DrawPitchLadder(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 clipRadius,
    keel::f32 pitchDeg,
    keel::f32 horizonRollDeg,
    keel::f32 pxPerDeg,
    const AttitudePalette& pal,
    bool isHud
) noexcept {
    const keel::f32 phi = keel::deg_to_rad(horizonRollDeg);
    // Unit vector along horizon (left to right when unrolled)
    const Vec2 u{keel::cos(phi), -keel::sin(phi)};
    // Unit normal pointing into ground (downward when unrolled)
    const Vec2 n{keel::sin(phi), keel::cos(phi)};

    const keel::f32 ladderClipR = clipRadius * 0.78f;

    auto toScreen = [&](keel::f32 lateralPx, keel::f32 ladderPitchDeg) noexcept -> Vec2 {
        const keel::f32 distAlongNormal = (pitchDeg - ladderPitchDeg) * pxPerDeg;
        return Vec2{
            center.x + lateralPx * u.x + distAlongNormal * n.x,
            center.y + lateralPx * u.y + distAlongNormal * n.y
        };
    };

    auto drawClippedLine = [&](Vec2 a, Vec2 b, Color col, keel::f32 w) noexcept {
        if (ClipSegmentToCircle(a, b, center, ladderClipR)) {
            canvas.DrawLine(a, b, col, w);
        }
    };

    for (keel::i32 halfSteps = -34; halfSteps <= 34; ++halfSteps) {
        if (halfSteps == 0) continue; // Horizon line is drawn across full sphere
        const keel::f32 deg = keel::as<keel::f32>(halfSteps) * 2.5f;
        if (keel::abs(deg - pitchDeg) * pxPerDeg > ladderClipR * 1.15f) continue;

        const bool isTen = (halfSteps % 4 == 0);
        const bool isFive = (halfSteps % 2 == 0) && !isTen;
        const bool isTwoHalf = !isTen && !isFive;

        if (isTwoHalf && keel::abs(deg) > 20.0f) continue;

        const keel::f32 innerGap = isTen ? (clipRadius * 0.12f) : (isFive ? (clipRadius * 0.09f) : (clipRadius * 0.05f));
        const keel::f32 outerSpan = isTen ? (clipRadius * 0.36f) : (isFive ? (clipRadius * 0.22f) : (clipRadius * 0.12f));
        const keel::f32 tipLen = isTen ? (clipRadius * 0.055f) : (isFive ? (clipRadius * 0.04f) : 0.0f);
        const keel::f32 lineW = isTen ? 1.8f : 1.3f;

        // Tips point toward the horizon: for deg > 0, toward +n (downward); for deg < 0, toward -n (upward)
        const keel::f32 tipPitchDelta = (deg > 0.0f) ? (-tipLen / pxPerDeg) : (+tipLen / pxPerDeg);

        for (keel::i32 side = -1; side <= 1; side += 2) {
            const keel::f32 s = keel::as<keel::f32>(side);
            const keel::f32 xInner = s * innerGap;
            const keel::f32 xOuter = s * outerSpan;

            if (deg > 0.0f || isTwoHalf) {
                // Solid pitch bar above horizon (or minor 2.5 deg tick)
                drawClippedLine(toScreen(xInner, deg), toScreen(xOuter, deg), pal.pitchLadder, lineW);
            } else {
                // Dashed pitch bar below horizon (2 segments per side)
                const keel::f32 mid1 = xInner + (xOuter - xInner) * 0.42f;
                const keel::f32 mid2 = xInner + (xOuter - xInner) * 0.58f;
                drawClippedLine(toScreen(xInner, deg), toScreen(mid1, deg), pal.pitchLadder, lineW);
                drawClippedLine(toScreen(mid2, deg), toScreen(xOuter, deg), pal.pitchLadder, lineW);
            }

            if (tipLen > 0.5f) {
                // Vertical horizon-pointing tip at outer end (or inner end in HUD mode)
                const keel::f32 tipX = isHud ? xInner : xOuter;
                drawClippedLine(toScreen(tipX, deg), toScreen(tipX, deg + tipPitchDelta), pal.pitchLadder, lineW);
            }
        }

        // Degree labels on both sides of 10-degree bars
        if (isTen) {
            const keel::i32 absDeg = keel::round_i32(keel::abs(deg));
            char buf[8];
            const keel::StringRef lbl = keel::format_buf(buf, "%d", absDeg);
            const keel::f32 fontSize = keel::clamp(clipRadius * 0.072f, 9.5f, 12.5f);
            const Vec2 textSz = canvas.MeasureText(lbl, fontSize);

            for (keel::i32 side = -1; side <= 1; side += 2) {
                const keel::f32 labelX = keel::as<keel::f32>(side) * (outerSpan + textSz.x * 0.7f + 5.0f);
                const Vec2 pt = toScreen(labelX, deg);
                const keel::f32 dx = pt.x - center.x;
                const keel::f32 dy = pt.y - center.y;
                if (dx * dx + dy * dy <= ladderClipR * ladderClipR) {
                    canvas.DrawText(lbl, {pt.x - textSz.x * 0.5f, pt.y - textSz.y * 0.5f}, fontSize, pal.pitchLadder);
                }
            }
        }

        // Unusual Attitude Recovery Chevrons at +-35, +-45, +-65 deg pointing toward horizon
        if (halfSteps == 14 || halfSteps == -14 || halfSteps == 18 || halfSteps == -18 || halfSteps == 26 || halfSteps == -26) {
            const keel::f32 chevW = clipRadius * 0.14f;
            const keel::f32 chevH = (deg > 0.0f) ? -3.2f : +3.2f; // Points toward horizon (0 deg)
            const Vec2 apex = toScreen(0.0f, deg + chevH);
            const Vec2 left = toScreen(-chevW, deg);
            const Vec2 right = toScreen(+chevW, deg);
            drawClippedLine(left, apex, pal.chevronAlert, 2.4f);
            drawClippedLine(right, apex, pal.chevronAlert, 2.4f);
        }
    }

    // Zenith (+90 deg) and Nadir (-90 deg) reticles
    for (keel::i32 pole = -1; pole <= 1; pole += 2) {
        const keel::f32 poleDeg = keel::as<keel::f32>(pole) * 90.0f;
        const Vec2 pt = toScreen(0.0f, poleDeg);
        const keel::f32 dx = pt.x - center.x;
        const keel::f32 dy = pt.y - center.y;
        if (dx * dx + dy * dy < ladderClipR * ladderClipR) {
            const keel::f32 r = clipRadius * 0.055f;
            DrawCircleOutline(canvas, pt, r, pal.pitchLadder, 1.5f, 20);
            if (pole > 0) {
                canvas.DrawLine({pt.x - r * 1.4f, pt.y}, {pt.x + r * 1.4f, pt.y}, pal.pitchLadder, 1.3f);
                canvas.DrawLine({pt.x, pt.y - r * 1.4f}, {pt.x, pt.y + r * 1.4f}, pal.pitchLadder, 1.3f);
            }
        }
    }
}

/// Draws the upper Bank Scale Arc (10, 20, 30, 45, 60 deg), rotating Roll Pointer, and Slip/Skid Inclinometer.
void DrawBankScale(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 radius,
    keel::f32 rollDeg,
    keel::f32 slipSkid,
    HorizonReferenceMode refMode,
    const AttitudePalette& pal
) noexcept {
    const keel::f32 arcR = radius * 0.85f;

    // 1. Upper bank arc from -60 to +60 deg (where 0 deg is straight up at -pi/2)
    constexpr keel::i32 kArcSegments = 36;
    Vec2 prev{
        center.x + keel::sin(keel::deg_to_rad(-60.0f)) * arcR,
        center.y - keel::cos(keel::deg_to_rad(-60.0f)) * arcR
    };
    for (keel::i32 i = 1; i <= kArcSegments; ++i) {
        const keel::f32 deg = -60.0f + (keel::as<keel::f32>(i) / kArcSegments) * 120.0f;
        const keel::f32 rad = keel::deg_to_rad(deg);
        const Vec2 curr{
            center.x + keel::sin(rad) * arcR,
            center.y - keel::cos(rad) * arcR
        };
        canvas.DrawLine(prev, curr, pal.bankScale, 1.5f);
        prev = curr;
    }

    // 2. Bank scale radial ticks at +-10, +-20, +-30, +-45, +-60 deg
    constexpr keel::StaticArray<keel::i32, 10> kTicks = {-60, -45, -30, -20, -10, 10, 20, 30, 45, 60};
    for (keel::i32 tDeg : kTicks) {
        const keel::f32 rad = keel::deg_to_rad(keel::as<keel::f32>(tDeg));
        const keel::f32 s = keel::sin(rad);
        const keel::f32 c = keel::cos(rad);
        const bool isMajor = (keel::abs(tDeg) == 30 || keel::abs(tDeg) == 60);
        const bool isFortyFive = (keel::abs(tDeg) == 45);
        const keel::f32 tickLen = isMajor ? (radius * 0.068f) : (radius * 0.038f);

        const Vec2 p0{center.x + s * arcR, center.y - c * arcR};
        const Vec2 p1{center.x + s * (arcR + tickLen), center.y - c * (arcR + tickLen)};

        if (isFortyFive) {
            // Small hollow triangle/diamond at 45 deg
            const Vec2 tip = p0;
            const Vec2 ortho{c, s};
            const keel::f32 halfW = radius * 0.018f;
            const Vec2 bL{p1.x - ortho.x * halfW, p1.y - ortho.y * halfW};
            const Vec2 bR{p1.x + ortho.x * halfW, p1.y + ortho.y * halfW};
            canvas.DrawTriangle(tip, bL, bR, pal.bankScale);
        } else {
            canvas.DrawLine(p0, p1, pal.bankScale, isMajor ? 2.0f : 1.4f);
        }
    }

    // 3. Fixed top 0-degree sky reference triangle (pointing down to arcR)
    const keel::f32 triH = radius * 0.052f;
    const keel::f32 triW = radius * 0.032f;
    const Vec2 fixedApex{center.x, center.y - arcR};
    const Vec2 fixedTL{center.x - triW, center.y - arcR - triH};
    const Vec2 fixedTR{center.x + triW, center.y - arcR - triH};
    canvas.DrawTriangle(fixedApex, fixedTL, fixedTR, pal.aircraftFill, pal.aircraftStroke, 1.0f);

    // 4. Moving Roll Pointer Triangle & Slip/Skid Trapezoid ("Ball") inside the arc
    const keel::f32 pointerAngleDeg = (refMode == HorizonReferenceMode::WesternMovingHorizon) ? -rollDeg : rollDeg;
    const keel::f32 pRad = keel::deg_to_rad(pointerAngleDeg);
    const Vec2 radialUp{keel::sin(pRad), -keel::cos(pRad)};
    const Vec2 tangent{keel::cos(pRad), keel::sin(pRad)};

    const keel::f32 pTipR = arcR - 2.0f;
    const keel::f32 pBaseR = pTipR - triH;
    const Vec2 pApex{center.x + radialUp.x * pTipR, center.y + radialUp.y * pTipR};
    const Vec2 pBaseL{
        center.x + radialUp.x * pBaseR - tangent.x * triW,
        center.y + radialUp.y * pBaseR - tangent.y * triW
    };
    const Vec2 pBaseR_pt{
        center.x + radialUp.x * pBaseR + tangent.x * triW,
        center.y + radialUp.y * pBaseR + tangent.y * triW
    };
    const Color ptrColor = ( keel::abs(rollDeg) > 45.0f ) ? pal.amberWarn : pal.bankScale;
    canvas.DrawTriangle(pApex, pBaseL, pBaseR_pt, ptrColor, pal.aircraftStroke, 1.0f);

    // Slip/Skid Trapezoid directly beneath the roll pointer base
    const keel::f32 slipPx = keel::clamp(slipSkid, -1.0f, 1.0f) * (triW * 2.4f);
    const keel::f32 trapTopR = pBaseR - 2.5f;
    const keel::f32 trapBotR = trapTopR - radius * 0.024f;
    const keel::f32 topW = triW * 0.88f;
    const keel::f32 botW = triW * 1.08f;

    const Vec2 t0{
        center.x + radialUp.x * trapTopR + tangent.x * (slipPx - topW),
        center.y + radialUp.y * trapTopR + tangent.y * (slipPx - topW)
    };
    const Vec2 t1{
        center.x + radialUp.x * trapTopR + tangent.x * (slipPx + topW),
        center.y + radialUp.y * trapTopR + tangent.y * (slipPx + topW)
    };
    const Vec2 t2{
        center.x + radialUp.x * trapBotR + tangent.x * (slipPx + botW),
        center.y + radialUp.y * trapBotR + tangent.y * (slipPx + botW)
    };
    const Vec2 t3{
        center.x + radialUp.x * trapBotR + tangent.x * (slipPx - botW),
        center.y + radialUp.y * trapBotR + tangent.y * (slipPx - botW)
    };
    DrawQuadFilled(canvas, t0, t1, t2, t3, ptrColor, pal.aircraftStroke, 1.0f);
}

/// Draws Flight Path Vector ("bird") and Flight Director (Single-Cue V-Bar or Dual-Cue Crossbars).
void DrawFlightDirectorAndFpv(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 radius,
    const AttitudeViewOptions& opts,
    keel::f32 pxPerDeg,
    const AttitudePalette& pal
) noexcept {
    // 1. Flight Path Vector (FPV / Velocity Vector)
    if (opts.showFlightPathVector) {
        const keel::f32 fpvPitch = opts.pitchDeg - opts.aoaDeg;
        const keel::f32 dy = (opts.pitchDeg - fpvPitch) * pxPerDeg; // +aoaDeg shifts bird down when nose is pitched above velocity vector
        const keel::f32 dx = opts.sideslipDeg * pxPerDeg;
        const Vec2 fpvPos{
            keel::clamp(center.x + dx, center.x - radius * 0.7f, center.x + radius * 0.7f),
            keel::clamp(center.y + dy, center.y - radius * 0.7f, center.y + radius * 0.7f)
        };
        const keel::f32 r = keel::clamp(radius * 0.042f, 5.5f, 9.0f);
        DrawCircleOutline(canvas, fpvPos, r, pal.fpvColor, 2.0f, 24);
        // Left & right horizontal wings
        canvas.DrawLine({fpvPos.x - r * 2.5f, fpvPos.y}, {fpvPos.x - r, fpvPos.y}, pal.fpvColor, 2.0f);
        canvas.DrawLine({fpvPos.x + r, fpvPos.y}, {fpvPos.x + r * 2.5f, fpvPos.y}, pal.fpvColor, 2.0f);
        // Top vertical stabilizer fin
        canvas.DrawLine({fpvPos.x, fpvPos.y - r}, {fpvPos.x, fpvPos.y - r * 2.1f}, pal.fpvColor, 2.0f);
    }

    // 2. Flight Director (FD)
    if (opts.flightDirectorStyle == FlightDirectorStyle::Off) return;

    const keel::f32 fdDy = (opts.pitchDeg - opts.fdPitchDeg) * pxPerDeg;
    const keel::f32 fdRollDiff = (opts.fdRollDeg - opts.rollDeg);

    if (opts.flightDirectorStyle == FlightDirectorStyle::SingleCueVBar) {
        // Magenta V-Bar command wedges
        const Vec2 apex{
            center.x,
            keel::clamp(center.y + fdDy, center.y - radius * 0.62f, center.y + radius * 0.62f)
        };
        const keel::f32 rotRad = keel::deg_to_rad(fdRollDiff);
        const Vec2 u{keel::cos(rotRad), keel::sin(rotRad)};
        const Vec2 n{-keel::sin(rotRad), keel::cos(rotRad)};

        const keel::f32 wingSpan = radius * 0.34f;
        const keel::f32 wingDrop = radius * 0.085f;
        const keel::f32 wingThick = radius * 0.036f;

        for (keel::i32 side = -1; side <= 1; side += 2) {
            const keel::f32 s = keel::as<keel::f32>(side);
            const Vec2 pTip{
                apex.x + s * (radius * 0.035f) * u.x,
                apex.y + s * (radius * 0.035f) * u.y
            };
            const Vec2 pOuterTop{
                apex.x + s * wingSpan * u.x + (wingDrop - wingThick) * n.x,
                apex.y + s * wingSpan * u.y + (wingDrop - wingThick) * n.y
            };
            const Vec2 pOuterBot{
                apex.x + s * wingSpan * u.x + (wingDrop + wingThick) * n.x,
                apex.y + s * wingSpan * u.y + (wingDrop + wingThick) * n.y
            };
            canvas.DrawTriangle(pTip, pOuterTop, pOuterBot, pal.flightDirector, pal.aircraftStroke, 1.2f);
        }
    } else if (opts.flightDirectorStyle == FlightDirectorStyle::DualCueCrossbars) {
        // Dual-cue horizontal pitch bar & vertical roll bar
        const keel::f32 barSpan = radius * 0.42f;
        const keel::f32 pitchY = keel::clamp(center.y + fdDy, center.y - barSpan, center.y + barSpan);
        const keel::f32 rollX = keel::clamp(center.x + fdRollDiff * pxPerDeg * 0.7f, center.x - barSpan, center.x + barSpan);

        canvas.DrawLine({center.x - barSpan, pitchY}, {center.x + barSpan, pitchY}, pal.aircraftStroke, 4.4f);
        canvas.DrawLine({center.x - barSpan, pitchY}, {center.x + barSpan, pitchY}, pal.flightDirector, 2.6f);

        canvas.DrawLine({rollX, center.y - barSpan}, {rollX, center.y + barSpan}, pal.aircraftStroke, 4.4f);
        canvas.DrawLine({rollX, center.y - barSpan}, {rollX, center.y + barSpan}, pal.flightDirector, 2.6f);
    }
}

/// Draws the central Aircraft Reference Symbol (GullWing, ClassicW, or RussianSilhouette).
void DrawAircraftReferenceSymbol(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 radius,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    // In WesternMovingHorizon, aircraft symbol stays level (0 deg).
    // In EasternMovingAircraft (KPPI), aircraft symbol rotates by +rollDeg.
    const keel::f32 symRollDeg = (opts.referenceMode == HorizonReferenceMode::EasternMovingAircraft)
        ? opts.rollDeg
        : 0.0f;
    const keel::f32 rad = keel::deg_to_rad(symRollDeg);
    const Vec2 u{keel::cos(rad), keel::sin(rad)};
    const Vec2 n{-keel::sin(rad), keel::cos(rad)};

    auto pt = [&](keel::f32 x, keel::f32 y) noexcept -> Vec2 {
        return Vec2{
            center.x + x * u.x + y * n.x,
            center.y + x * u.y + y * n.y
        };
    };

    if (opts.aircraftSymbol == AircraftSymbolStyle::GullWing) {
        // Center bore-sight square
        const keel::f32 sq = keel::max(3.5f, radius * 0.025f);
        DrawQuadFilled(canvas, pt(-sq, -sq), pt(sq, -sq), pt(sq, sq), pt(-sq, sq), pal.aircraftFill, pal.aircraftStroke, 1.4f);

        // Left & Right L-shaped gull wings
        const keel::f32 innerX = radius * 0.14f;
        const keel::f32 outerX = radius * 0.38f;
        const keel::f32 barH = keel::max(3.2f, radius * 0.022f);
        const keel::f32 dropH = radius * 0.065f;

        for (keel::i32 side = -1; side <= 1; side += 2) {
            const keel::f32 s = keel::as<keel::f32>(side);
            // Horizontal wing bar
            DrawQuadFilled(
                canvas,
                pt(s * innerX, -barH),
                pt(s * outerX, -barH),
                pt(s * outerX, +barH),
                pt(s * innerX, +barH),
                pal.aircraftFill,
                pal.aircraftStroke,
                1.3f
            );
            // Inner vertical drop tab
            DrawQuadFilled(
                canvas,
                pt(s * innerX, +barH),
                pt(s * (innerX + barH * 2.0f), +barH),
                pt(s * (innerX + barH * 2.0f), +dropH),
                pt(s * innerX, +dropH),
                pal.aircraftFill,
                pal.aircraftStroke,
                1.3f
            );
        }
    } else if (opts.aircraftSymbol == AircraftSymbolStyle::ClassicW) {
        const keel::StaticArray<Vec2, 7> wPts = {
            pt(-radius * 0.36f, 0.0f),
            pt(-radius * 0.16f, 0.0f),
            pt(-radius * 0.08f, radius * 0.07f),
            pt(0.0f, 0.0f),
            pt(+radius * 0.08f, radius * 0.07f),
            pt(+radius * 0.16f, 0.0f),
            pt(+radius * 0.36f, 0.0f)
        };
        for (keel::usize i = 0; i + 1 < wPts.size(); ++i) {
            canvas.DrawLine(wPts[i], wPts[i + 1], pal.aircraftStroke, 5.0f);
        }
        for (keel::usize i = 0; i + 1 < wPts.size(); ++i) {
            canvas.DrawLine(wPts[i], wPts[i + 1], pal.aircraftFill, 2.8f);
        }
    } else {
        // Russian KPPI silhouette: wide wings + fuselage ring + vertical tail fin
        const keel::f32 rCenter = keel::max(5.0f, radius * 0.042f);
        const keel::f32 wingOuter = radius * 0.42f;
        canvas.DrawLine(pt(-wingOuter, 0.0f), pt(-rCenter, 0.0f), pal.aircraftStroke, 5.2f);
        canvas.DrawLine(pt(+rCenter, 0.0f), pt(+wingOuter, 0.0f), pal.aircraftStroke, 5.2f);
        canvas.DrawLine(pt(0.0f, -rCenter), pt(0.0f, -radius * 0.14f), pal.aircraftStroke, 5.2f);

        canvas.DrawLine(pt(-wingOuter, 0.0f), pt(-rCenter, 0.0f), pal.aircraftFill, 3.0f);
        canvas.DrawLine(pt(+rCenter, 0.0f), pt(+wingOuter, 0.0f), pal.aircraftFill, 3.0f);
        canvas.DrawLine(pt(0.0f, -rCenter), pt(0.0f, -radius * 0.14f), pal.aircraftFill, 3.0f);
        DrawCircleOutline(canvas, center, rCenter, pal.aircraftFill, 2.6f, 24);
        DrawFilledCircle(canvas, center, 2.2f, pal.aircraftFill);
    }
}

/// Draws the left Indicated Airspeed (IAS) scrolling tape, bug, speed limits, and Mach box.
void DrawAirspeedTape(
    ICanvas& canvas,
    Rect tapeRect,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    if (tapeRect.w < 24.0f || tapeRect.h < 60.0f) return;

    const keel::f32 headerH = 22.0f;
    const keel::f32 footerH = 22.0f;
    const Rect bodyRect(tapeRect.x, tapeRect.y + headerH, tapeRect.w, keel::max(30.0f, tapeRect.h - headerH - footerH));
    const keel::f32 midY = bodyRect.y + bodyRect.h * 0.5f;
    const keel::f32 pxPerKnot = bodyRect.h / 90.0f; // 90 knots visible span (+-45 kt)

    // Top selected airspeed bug readout box
    const Rect topBox(tapeRect.x, tapeRect.y, tapeRect.w, headerH - 2.0f);
    canvas.DrawRoundRect(topBox, 3.0f, Color(9, 13, 24, 235), pal.tapeBorder, 1.0f);
    char bugBuf[24];
    canvas.DrawTextBox(keel::format_buf(bugBuf, "%03.0f", keel::max(0.0f, opts.targetAirspeedBug)), topBox, 11.5f, pal.bugCyan, 0.5f, 0.5f);

    // Main scrolling tape body
    canvas.DrawRoundRect(bodyRect, 2.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
    canvas.PushClip(bodyRect);

    const keel::f32 curIas = keel::max(0.0f, opts.airspeedKnots);
    const keel::i32 minKt = keel::max(0, keel::floor_i32((curIas - 55.0f) / 5.0f) * 5);
    const keel::i32 maxKt = keel::ceil_i32((curIas + 55.0f) / 5.0f) * 5;

    // Speed envelope colored strips along right edge of tape (Red stall < 110 kt, Green 135..310 kt, Red Vmo > 330 kt)
    auto ktToY = [&](keel::f32 kt) noexcept -> keel::f32 {
        return midY - (kt - curIas) * pxPerKnot;
    };

    const keel::f32 stripX = bodyRect.Right() - 4.5f;
    const keel::f32 yStallTop = keel::clamp(ktToY(115.0f), bodyRect.y, bodyRect.Bottom());
    const keel::f32 yStallBot = keel::clamp(ktToY(0.0f), bodyRect.y, bodyRect.Bottom());
    if (yStallBot > yStallTop) {
        canvas.DrawRect(Rect(stripX, yStallTop, 3.5f, yStallBot - yStallTop), pal.redLimit);
    }

    const keel::f32 yGreenTop = keel::clamp(ktToY(315.0f), bodyRect.y, bodyRect.Bottom());
    const keel::f32 yGreenBot = keel::clamp(ktToY(135.0f), bodyRect.y, bodyRect.Bottom());
    if (yGreenBot > yGreenTop) {
        canvas.DrawRect(Rect(stripX, yGreenTop, 2.5f, yGreenBot - yGreenTop), pal.greenArc);
    }

    const keel::f32 yVmoTop = keel::clamp(ktToY(999.0f), bodyRect.y, bodyRect.Bottom());
    const keel::f32 yVmoBot = keel::clamp(ktToY(330.0f), bodyRect.y, bodyRect.Bottom());
    if (yVmoBot > yVmoTop) {
        canvas.DrawRect(Rect(stripX, yVmoTop, 3.5f, yVmoBot - yVmoTop), pal.redLimit);
    }

    // Ticks every 5 knots, numbers every 10 knots
    for (keel::i32 kt = minKt; kt <= maxKt; kt += 5) {
        const keel::f32 y = ktToY(keel::as<keel::f32>(kt));
        if (y < bodyRect.y - 8.0f || y > bodyRect.Bottom() + 8.0f) continue;

        const bool isTen = (kt % 10 == 0);
        const keel::f32 tickW = isTen ? 9.0f : 5.0f;
        canvas.DrawLine({bodyRect.Right() - tickW, y}, {bodyRect.Right(), y}, pal.tapeTick, isTen ? 1.4f : 1.0f);

        if (isTen) {
            char kBuf[16];
            canvas.DrawTextBox(
                keel::format_buf(kBuf, "%d", kt),
                Rect(bodyRect.x + 3.0f, y - 8.0f, bodyRect.w - tickW - 6.0f, 16.0f),
                10.5f,
                pal.tapeTick,
                1.0f,
                0.5f
            );
        }
    }

    // Autopilot Selected Speed Bug on right edge
    const keel::f32 bugY = keel::clamp(ktToY(opts.targetAirspeedBug), bodyRect.y + 5.0f, bodyRect.Bottom() - 5.0f);
    canvas.DrawTriangle(
        {bodyRect.Right() - 8.0f, bugY},
        {bodyRect.Right(), bugY - 5.5f},
        {bodyRect.Right(), bugY + 5.5f},
        pal.bugCyan
    );

    // Center Rolling-Drum Digital Value Box
    const keel::f32 drumH = 24.0f;
    const Rect drumRect(bodyRect.x + 1.0f, midY - drumH * 0.5f, bodyRect.w - 6.0f, drumH);
    canvas.DrawRoundRect(drumRect, 2.0f, Color(6, 9, 16, 250), pal.fmaWhite, 1.5f);
    canvas.DrawTriangle(
        {bodyRect.Right() - 1.0f, midY},
        {drumRect.Right(), midY - 5.0f},
        {drumRect.Right(), midY + 5.0f},
        pal.fmaWhite
    );
    char curBuf[16];
    canvas.DrawTextBox(keel::format_buf(curBuf, "%03.0f", curIas), drumRect, 13.0f, pal.fmaWhite, 0.5f, 0.5f);

    canvas.PopClip();

    // Bottom Mach Number Box
    const Rect botBox(tapeRect.x, bodyRect.Bottom() + 2.0f, tapeRect.w, footerH - 2.0f);
    canvas.DrawRoundRect(botBox, 3.0f, Color(9, 13, 24, 235), pal.tapeBorder, 1.0f);
    char machBuf[24];
    canvas.DrawTextBox(
        keel::format_buf(machBuf, "M .%03d", keel::clamp(keel::round_i32(opts.machNumber * 1000.0f), 0, 999)),
        botBox,
        10.5f,
        pal.fmaGreen,
        0.5f,
        0.5f
    );
}

/// Draws the right Barometric Altitude scrolling tape, Baro box, and Vertical Speed Indicator (VSI) column.
void DrawAltitudeAndVsiTape(
    ICanvas& canvas,
    Rect altGroupRect,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    if (altGroupRect.w < 32.0f || altGroupRect.h < 60.0f) return;

    const keel::f32 vsiW = keel::clamp(altGroupRect.w * 0.30f, 22.0f, 32.0f);
    const keel::f32 altW = keel::max(28.0f, altGroupRect.w - vsiW - 3.0f);
    const keel::f32 headerH = 22.0f;
    const keel::f32 footerH = 22.0f;

    const Rect altBody(altGroupRect.x, altGroupRect.y + headerH, altW, keel::max(30.0f, altGroupRect.h - headerH - footerH));
    const keel::f32 midY = altBody.y + altBody.h * 0.5f;
    const keel::f32 pxPerFoot = altBody.h / 900.0f; // +-450 ft visible span

    // 1. Top Selected Altitude Bug Readout Box
    const Rect topBox(altGroupRect.x, altGroupRect.y, altGroupRect.w, headerH - 2.0f);
    canvas.DrawRoundRect(topBox, 3.0f, Color(9, 13, 24, 235), pal.tapeBorder, 1.0f);
    char altBugBuf[24];
    canvas.DrawTextBox(keel::format_buf(altBugBuf, "%05.0f", opts.targetAltitudeBug), topBox, 11.5f, pal.bugCyan, 0.5f, 0.5f);

    // 2. Main Scrolling Altitude Tape
    canvas.DrawRoundRect(altBody, 2.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
    canvas.PushClip(altBody);

    const keel::f32 curAlt = opts.altitudeFeet;
    const keel::i32 minFt = keel::floor_i32((curAlt - 550.0f) / 20.0f) * 20;
    const keel::i32 maxFt = keel::ceil_i32((curAlt + 550.0f) / 20.0f) * 20;

    auto ftToY = [&](keel::f32 ft) noexcept -> keel::f32 {
        return midY - (ft - curAlt) * pxPerFoot;
    };

    for (keel::i32 ft = minFt; ft <= maxFt; ft += 20) {
        const keel::f32 y = ftToY(keel::as<keel::f32>(ft));
        if (y < altBody.y - 8.0f || y > altBody.Bottom() + 8.0f) continue;

        const bool isHundred = (ft % 100 == 0);
        const keel::f32 tickW = isHundred ? 8.5f : 4.5f;
        canvas.DrawLine({altBody.x, y}, {altBody.x + tickW, y}, pal.tapeTick, isHundred ? 1.4f : 1.0f);

        if (isHundred) {
            char fBuf[16];
            canvas.DrawTextBox(
                keel::format_buf(fBuf, "%d", ft),
                Rect(altBody.x + tickW + 3.0f, y - 8.0f, altBody.w - tickW - 5.0f, 16.0f),
                10.0f,
                pal.tapeTick,
                0.0f,
                0.5f
            );
        }
    }

    // Selected Altitude Bug on left edge of altitude tape
    const keel::f32 bugY = keel::clamp(ftToY(opts.targetAltitudeBug), altBody.y + 5.0f, altBody.Bottom() - 5.0f);
    canvas.DrawTriangle(
        {altBody.x + 8.0f, bugY},
        {altBody.x, bugY - 5.5f},
        {altBody.x, bugY + 5.5f},
        pal.bugCyan
    );

    // Center Rolling-Drum Digital Altitude Readout Box
    const keel::f32 drumH = 24.0f;
    const Rect drumRect(altBody.x + 5.0f, midY - drumH * 0.5f, altBody.w - 6.0f, drumH);
    canvas.DrawRoundRect(drumRect, 2.0f, Color(6, 9, 16, 250), pal.fmaWhite, 1.5f);
    canvas.DrawTriangle(
        {altBody.x + 1.0f, midY},
        {drumRect.x, midY - 5.0f},
        {drumRect.x, midY + 5.0f},
        pal.fmaWhite
    );
    char curAltBuf[16];
    canvas.DrawTextBox(keel::format_buf(curAltBuf, "%05.0f", curAlt), drumRect, 12.0f, pal.fmaWhite, 0.5f, 0.5f);

    canvas.PopClip();

    // 3. Vertical Speed Indicator (VSI) Column on the right
    const Rect vsiRect(altBody.Right() + 3.0f, altBody.y + 12.0f, vsiW, keel::max(24.0f, altBody.h - 24.0f));
    canvas.DrawRoundRect(vsiRect, 2.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
    canvas.PushClip(vsiRect);

    const keel::f32 vsiMidY = vsiRect.y + vsiRect.h * 0.5f;
    const keel::f32 vsiHalfH = (vsiRect.h * 0.5f) - 8.0f;

    // Piecewise non-linear VSI mapping (-4000 .. +4000 fpm)
    auto fpmToY = [&](keel::f32 fpm) noexcept -> keel::f32 {
        const keel::f32 clamped = keel::clamp(fpm, -4000.0f, 4000.0f);
        const keel::f32 sign = (clamped >= 0.0f) ? 1.0f : -1.0f;
        const keel::f32 a = keel::abs(clamped);
        keel::f32 norm = 0.0f;
        if (a <= 1000.0f) {
            norm = (a / 1000.0f) * 0.45f;
        } else if (a <= 2000.0f) {
            norm = 0.45f + ((a - 1000.0f) / 1000.0f) * 0.30f;
        } else {
            norm = 0.75f + ((a - 2000.0f) / 2000.0f) * 0.25f;
        }
        return vsiMidY - sign * norm * vsiHalfH;
    };

    constexpr keel::StaticArray<keel::i32, 7> kVsiMarks = {-4000, -2000, -1000, 0, 1000, 2000, 4000};
    for (keel::i32 fpm : kVsiMarks) {
        const keel::f32 y = fpmToY(keel::as<keel::f32>(fpm));
        const bool isZero = (fpm == 0);
        canvas.DrawLine({vsiRect.x, y}, {vsiRect.x + (isZero ? 8.0f : 5.0f), y}, pal.tapeTick, isZero ? 1.6f : 1.1f);
        if (fpm != 0) {
            char vBuf[8];
            canvas.DrawText(keel::format_buf(vBuf, "%d", keel::abs(fpm) / 1000), {vsiRect.x + 6.5f, y - 5.5f}, 9.0f, pal.tapeTick);
        }
    }

    // VSI Needle from right edge to current vertical speed Y
    const keel::f32 needleY = fpmToY(opts.verticalSpeedFpm);
    canvas.DrawLine({vsiRect.Right() - 2.0f, vsiMidY}, {vsiRect.x + 2.0f, needleY}, pal.fmaGreen, 2.2f);
    canvas.PopClip();

    // 4. Bottom Barometric Setting Box (29.92 IN or STD)
    const Rect botBox(altGroupRect.x, altBody.Bottom() + 2.0f, altGroupRect.w, footerH - 2.0f);
    canvas.DrawRoundRect(botBox, 3.0f, Color(9, 13, 24, 235), pal.tapeBorder, 1.0f);
    char baroBuf[24];
    const keel::StringRef baroRef = opts.baroStd
        ? keel::StringRef("STD BARO")
        : keel::format_buf(baroBuf, "%.2f IN", opts.baroSettingInHg);
    canvas.DrawTextBox(baroRef, botBox, 10.5f, pal.bugCyan, 0.5f, 0.5f);
}

/// Draws the bottom magnetic compass Heading Tape and Selected Heading Bug.
void DrawHeadingTape(
    ICanvas& canvas,
    Rect hdgRect,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    if (hdgRect.w < 60.0f || hdgRect.h < 18.0f) return;

    canvas.DrawRoundRect(hdgRect, 2.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
    canvas.PushClip(hdgRect);

    const keel::f32 curHdg = NormalizeHeadingDeg(opts.headingDeg);
    const keel::f32 midX = hdgRect.x + hdgRect.w * 0.5f;
    const keel::f32 pxPerDeg = hdgRect.w / 70.0f; // +-35 deg visible compass span

    const keel::i32 baseFive = keel::floor_i32(curHdg / 5.0f) * 5;
    for (keel::i32 offset = -40; offset <= 40; offset += 5) {
        const keel::i32 rawDeg = baseFive + offset;
        const keel::f32 diff = SignedAngleDiffDeg(keel::as<keel::f32>(rawDeg), curHdg);
        const keel::f32 x = midX + diff * pxPerDeg;
        if (x < hdgRect.x - 10.0f || x > hdgRect.Right() + 10.0f) continue;

        const keel::i32 normDeg = ((rawDeg % 360) + 360) % 360;
        const bool isTen = (normDeg % 10 == 0);
        const bool isCardinal = (normDeg % 90 == 0);
        const keel::f32 tickH = isTen ? 7.5f : 4.0f;

        canvas.DrawLine({x, hdgRect.y}, {x, hdgRect.y + tickH}, pal.tapeTick, isTen ? 1.4f : 1.0f);

        if (isTen) {
            char hBuf[12];
            keel::StringRef hRef{};
            if (normDeg == 0) hRef = "N";
            else if (normDeg == 90) hRef = "E";
            else if (normDeg == 180) hRef = "S";
            else if (normDeg == 270) hRef = "W";
            else hRef = keel::format_buf(hBuf, "%02d", normDeg / 10);

            canvas.DrawTextBox(
                hRef,
                Rect(x - 14.0f, hdgRect.y + tickH + 1.0f, 28.0f, hdgRect.h - tickH - 2.0f),
                isCardinal ? 11.0f : 10.0f,
                isCardinal ? pal.fmaCyan : pal.tapeTick,
                0.5f,
                0.5f
            );
        }
    }

    // Selected Heading Bug on top edge
    const keel::f32 bugDiff = SignedAngleDiffDeg(opts.targetHeadingBug, curHdg);
    const keel::f32 bugX = keel::clamp(midX + bugDiff * pxPerDeg, hdgRect.x + 6.0f, hdgRect.Right() - 6.0f);
    canvas.DrawTriangle(
        {bugX, hdgRect.y + 8.0f},
        {bugX - 5.5f, hdgRect.y},
        {bugX + 5.5f, hdgRect.y},
        pal.bugCyan
    );

    // Fixed Center Lubber Triangle & Digital Box
    canvas.DrawTriangle(
        {midX, hdgRect.y + 6.5f},
        {midX - 5.0f, hdgRect.y},
        {midX + 5.0f, hdgRect.y},
        pal.aircraftFill
    );

    canvas.PopClip();
}

/// Draws ILS Localizer (bottom) and Glideslope (right) 4-dot deviation scales and magenta diamonds.
void DrawIlsDeviationScales(
    ICanvas& canvas,
    Rect adiRect,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    const Vec2 c = adiRect.Center();

    // 1. Horizontal Localizer Scale near bottom of ADI
    const keel::f32 locY = adiRect.Bottom() - 12.0f;
    const keel::f32 locHalfW = keel::min(88.0f, adiRect.w * 0.34f);
    const Rect locBar(c.x - locHalfW - 8.0f, locY - 7.0f, (locHalfW + 8.0f) * 2.0f, 14.0f);
    canvas.DrawRoundRect(locBar, 4.0f, Color(9, 13, 24, 195), pal.tapeBorder, 1.0f);

    canvas.DrawLine({c.x, locY - 5.0f}, {c.x, locY + 5.0f}, pal.fmaWhite, 1.6f);
    for (keel::i32 dot = -2; dot <= 2; ++dot) {
        if (dot == 0) continue;
        const keel::f32 dx = (keel::as<keel::f32>(dot) / 2.0f) * locHalfW;
        DrawCircleOutline(canvas, {c.x + dx, locY}, 2.4f, pal.tapeTick, 1.2f, 12);
    }
    const keel::f32 locX = c.x + (keel::clamp(opts.locDeviationDots, -2.2f, 2.2f) / 2.0f) * locHalfW;
    DrawQuadFilled(
        canvas,
        {locX, locY - 5.0f},
        {locX + 5.5f, locY},
        {locX, locY + 5.0f},
        {locX - 5.5f, locY},
        pal.ilsMagenta
    );

    // 2. Vertical Glideslope Scale along right edge of ADI
    const keel::f32 gsX = adiRect.Right() - 12.0f;
    const keel::f32 gsHalfH = keel::min(88.0f, adiRect.h * 0.34f);
    const Rect gsBar(gsX - 7.0f, c.y - gsHalfH - 8.0f, 14.0f, (gsHalfH + 8.0f) * 2.0f);
    canvas.DrawRoundRect(gsBar, 4.0f, Color(9, 13, 24, 195), pal.tapeBorder, 1.0f);

    canvas.DrawLine({gsX - 5.0f, c.y}, {gsX + 5.0f, c.y}, pal.fmaWhite, 1.6f);
    for (keel::i32 dot = -2; dot <= 2; ++dot) {
        if (dot == 0) continue;
        const keel::f32 dy = (keel::as<keel::f32>(dot) / 2.0f) * gsHalfH;
        DrawCircleOutline(canvas, {gsX, c.y + dy}, 2.4f, pal.tapeTick, 1.2f, 12);
    }
    const keel::f32 gsY = c.y - (keel::clamp(opts.gsDeviationDots, -2.2f, 2.2f) / 2.0f) * gsHalfH;
    DrawQuadFilled(
        canvas,
        {gsX, gsY - 5.5f},
        {gsX + 5.0f, gsY},
        {gsX, gsY + 5.5f},
        {gsX - 5.0f, gsY},
        pal.ilsMagenta
    );
}

/// Draws the top 5-column Flight Mode Annunciator (FMA) status bar.
void DrawFmaBar(
    ICanvas& canvas,
    Rect fmaRect,
    const AttitudeViewOptions& opts,
    const AttitudePalette& pal
) noexcept {
    if (fmaRect.w < 120.0f || fmaRect.h < 18.0f) return;
    canvas.DrawRoundRect(fmaRect, 3.0f, pal.tapeBg, pal.tapeBorder, 1.0f);

    const keel::f32 colW = fmaRect.w / 5.0f;
    for (keel::i32 i = 1; i < 5; ++i) {
        const keel::f32 x = fmaRect.x + colW * keel::as<keel::f32>(i);
        canvas.DrawLine({x, fmaRect.y + 3.0f}, {x, fmaRect.Bottom() - 3.0f}, pal.tapeBorder, 1.0f);
    }

    const char* pitchMode = (keel::abs(opts.altitudeFeet - opts.targetAltitudeBug) < 80.0f) ? "ALT HOLD" : "VNAV PTH";
    const char* rollMode = (opts.showIlsDiamonds && keel::abs(opts.locDeviationDots) < 1.5f) ? "LOC*" : "HDG SEL";
    const char* fdLabel = (opts.flightDirectorStyle != FlightDirectorStyle::Off) ? "AP1  1FD2" : "MAN  A/THR";

    canvas.DrawTextBox("MACH / SPD", Rect(fmaRect.x, fmaRect.y, colW, fmaRect.h), 10.5f, pal.fmaGreen, 0.5f, 0.5f);
    canvas.DrawTextBox(pitchMode, Rect(fmaRect.x + colW, fmaRect.y, colW, fmaRect.h), 10.5f, pal.fmaGreen, 0.5f, 0.5f);
    canvas.DrawTextBox(rollMode, Rect(fmaRect.x + colW * 2.0f, fmaRect.y, colW, fmaRect.h), 10.5f, pal.fmaGreen, 0.5f, 0.5f);
    canvas.DrawTextBox("CAT III DUAL", Rect(fmaRect.x + colW * 3.0f, fmaRect.y, colW, fmaRect.h), 10.0f, pal.fmaCyan, 0.5f, 0.5f);
    canvas.DrawTextBox(fdLabel, Rect(fmaRect.x + colW * 4.0f, fmaRect.y, colW, fmaRect.h), 10.5f, pal.fmaWhite, 0.5f, 0.5f);
}

[[nodiscard]] AttitudeViewportMode ResolveViewportMode(const AttitudeViewOptions& opts) noexcept {
    if (opts.viewportMode != AttitudeViewportMode::Auto) {
        return opts.viewportMode;
    }
    return (opts.displayMode == AttitudeDisplayMode::CompactADI)
        ? AttitudeViewportMode::Circular
        : AttitudeViewportMode::RoundedDome;
}

struct AttitudeLayout {
    AttitudeViewportMode viewportMode{AttitudeViewportMode::RoundedDome};
    bool isFullPfd{false};
    bool isCompact{false};
    bool isHud{false};
    bool isFullBleed{false};
    bool isCircular{false};

    keel::f32 pad{8.0f};
    keel::f32 fmaH{0.0f};
    keel::f32 hdgH{0.0f};
    keel::f32 leftTapeW{0.0f};
    keel::f32 rightTapeW{0.0f};

    Rect availCenterRect{};
    Rect adiRect{};
    Rect horizonRect{};
    Vec2 adiCenter{};
    keel::f32 adiRadius{0.0f};
    keel::f32 horizonSpan{0.0f};
};

[[nodiscard]] AttitudeLayout ComputeAttitudeLayout(Rect r, const AttitudeViewOptions& opts) noexcept {
    AttitudeLayout lay{};
    lay.viewportMode = ResolveViewportMode(opts);
    lay.isFullPfd = (opts.displayMode == AttitudeDisplayMode::FullPFD);
    lay.isCompact = (opts.displayMode == AttitudeDisplayMode::CompactADI);
    lay.isHud = (opts.displayMode == AttitudeDisplayMode::HudOverlay);
    lay.isFullBleed = (lay.viewportMode == AttitudeViewportMode::FullBleed);
    lay.isCircular = (lay.viewportMode == AttitudeViewportMode::Circular);

    lay.pad = 8.0f;
    lay.fmaH = (opts.showFmaBar && !lay.isCompact && r.h > 220.0f) ? 26.0f : 0.0f;
    lay.hdgH = (opts.showHeadingTape && !lay.isCompact && r.h > 220.0f) ? 30.0f : 0.0f;
    lay.leftTapeW = (lay.isFullPfd && opts.showSpeedAndAltTapes && r.w > 300.0f)
        ? keel::clamp(r.w * 0.14f, 56.0f, 76.0f)
        : 0.0f;
    lay.rightTapeW = (lay.isFullPfd && opts.showSpeedAndAltTapes && r.w > 300.0f)
        ? keel::clamp(r.w * 0.19f, 76.0f, 106.0f)
        : 0.0f;

    const keel::f32 contentTop = r.y + lay.pad + (lay.fmaH > 0.0f ? (lay.fmaH + 6.0f) : 0.0f);
    const keel::f32 contentBot = r.Bottom() - lay.pad - (lay.hdgH > 0.0f ? (lay.hdgH + 6.0f) : 0.0f);
    const keel::f32 contentH = keel::max(40.0f, contentBot - contentTop);

    const keel::f32 adiLeft = r.x + lay.pad + (lay.leftTapeW > 0.0f ? (lay.leftTapeW + 8.0f) : 0.0f);
    const keel::f32 adiRight = r.Right() - lay.pad - (lay.rightTapeW > 0.0f ? (lay.rightTapeW + 8.0f) : 0.0f);
    const keel::f32 availAdiW = keel::max(40.0f, adiRight - adiLeft);

    lay.availCenterRect = Rect(adiLeft, contentTop, availAdiW, contentH);

    const keel::f32 adiSide = keel::min(availAdiW, contentH);
    lay.adiRect = Rect(
        adiLeft + (availAdiW - adiSide) * 0.5f,
        contentTop + (contentH - adiSide) * 0.5f,
        adiSide,
        adiSide
    );
    lay.horizonRect = lay.isFullBleed ? r : lay.adiRect;
    lay.adiCenter = lay.adiRect.Center();
    lay.adiRadius = adiSide * 0.5f;
    lay.horizonSpan = keel::hypot(lay.horizonRect.w, lay.horizonRect.h) * 1.25f;
    return lay;
}

void RenderAttitudeViewNode(const Node* node, ICanvas& canvas, const UiTheme& theme) noexcept {
    const auto* payload = node->As<AttitudeViewPayload>();
    if (!payload) return;

    const AttitudeViewOptions& opts = payload->opts;
    const AttitudeViewResponse& resp = payload->resp;

    const Rect r = node->computedRect;
    if (r.w <= 16.0f || r.h <= 16.0f) return;

    const AttitudePalette pal = BuildPalette(opts.colorScheme, opts.displayMode);
    const keel::f32 radius = opts.border.radius.has_value()
        ? *opts.border.radius
        : theme.GetRadius(node->uiSize);
    const Color bgConsole = opts.background.Resolve(node->hovered, node->pressed, node->disabled, pal.bgConsole);
    const Color borderCol = opts.border.color.Resolve(node->hovered, node->pressed, node->disabled, theme.palette.borderSubtle);
    const keel::f32 borderW = (opts.border.width > 0.0f) ? opts.border.width : 1.0f;

    canvas.DrawRoundRect(r, radius, bgConsole, borderCol, borderW);
    canvas.PushClip(r);

    const AttitudeLayout lay = ComputeAttitudeLayout(r, opts);
    const Rect adiRect = lay.adiRect;
    const Vec2 adiCenter = lay.adiCenter;
    const keel::f32 adiRadius = lay.adiRadius;

    // 1. Artificial Horizon Sphere (Sky & Ground Half-Plane Clipped Polygons)
    const keel::f32 horizonRollDeg = (opts.referenceMode == HorizonReferenceMode::WesternMovingHorizon)
        ? opts.rollDeg
        : 0.0f;
    const keel::f32 fovPitch = keel::clamp(resp.visibleFovPitchDeg, 25.0f, 90.0f);
    const keel::f32 pxPerDeg = (2.0f * adiRadius) / fovPitch;

    const keel::f32 phi = keel::deg_to_rad(horizonRollDeg);
    const Vec2 u{keel::cos(phi), -keel::sin(phi)};
    const Vec2 nGround{keel::sin(phi), keel::cos(phi)};
    const Vec2 nSky{-nGround.x, -nGround.y};

    const Vec2 horizonOrigin{
        adiCenter.x + (opts.pitchDeg * pxPerDeg) * nGround.x,
        adiCenter.y + (opts.pitchDeg * pxPerDeg) * nGround.y
    };

    const auto boundaryPoly = BuildAdiBoundaryPolygon(lay.horizonRect, lay.viewportMode, radius);

    if (!lay.isHud) {
        // Clip boundary polygon into Sky (nGround <= 0) and Ground (nSky <= 0)
        const auto skyPoly = ClipPolygonHalfPlane(boundaryPoly, horizonOrigin, nGround);
        const auto groundPoly = ClipPolygonHalfPlane(boundaryPoly, horizonOrigin, nSky);

        DrawConvexPolygonFilled(canvas, skyPoly, pal.skyFill);
        DrawConvexPolygonFilled(canvas, groundPoly, pal.groundFill);

        // Subtle atmospheric horizon glow bands (+/- 20 px around horizon)
        const keel::f32 glowBandPx = keel::min(22.0f, adiRadius * 0.16f);
        const Vec2 skyGlowLimit{
            horizonOrigin.x - glowBandPx * nGround.x,
            horizonOrigin.y - glowBandPx * nGround.y
        };
        const Vec2 groundGlowLimit{
            horizonOrigin.x + glowBandPx * nGround.x,
            horizonOrigin.y + glowBandPx * nGround.y
        };
        const auto skyGlowPoly = ClipPolygonHalfPlane(skyPoly, skyGlowLimit, nSky);
        const auto groundGlowPoly = ClipPolygonHalfPlane(groundPoly, groundGlowLimit, nGround);
        DrawConvexPolygonFilled(canvas, skyGlowPoly, pal.skyGlow);
        DrawConvexPolygonFilled(canvas, groundGlowPoly, pal.groundGlow);
    }

    // Horizon Dividing Line clipped to the active horizon boundary
    {
        Vec2 h0{horizonOrigin.x - lay.horizonSpan * u.x, horizonOrigin.y - lay.horizonSpan * u.y};
        Vec2 h1{horizonOrigin.x + lay.horizonSpan * u.x, horizonOrigin.y + lay.horizonSpan * u.y};
        if (lay.isCircular) {
            if (ClipSegmentToCircle(h0, h1, adiCenter, adiRadius)) {
                canvas.DrawLine(h0, h1, pal.horizonLine, 2.2f);
            }
        } else {
            canvas.PushClip(lay.horizonRect);
            canvas.DrawLine(h0, h1, pal.horizonLine, 2.2f);
            canvas.PopClip();
        }
    }

    // 2. Pitch Ladder, Zenith/Nadir Reticles & Recovery Chevrons
    const Rect symbologyClipRect = lay.isFullBleed ? lay.availCenterRect : adiRect;
    if (opts.showPitchLadder) {
        canvas.PushClip(symbologyClipRect);
        DrawPitchLadder(canvas, adiCenter, adiRadius, opts.pitchDeg, horizonRollDeg, pxPerDeg, pal, lay.isHud);
        canvas.PopClip();
    }

    // 3. Upper Bank Scale Arc, Roll Pointer & Slip/Skid Ball
    if (opts.showBankScale) {
        DrawBankScale(canvas, adiCenter, adiRadius, opts.rollDeg, opts.slipSkid, opts.referenceMode, pal);
    }

    // 4. Flight Path Vector (FPV) & Flight Director (FD)
    canvas.PushClip(symbologyClipRect);
    DrawFlightDirectorAndFpv(canvas, adiCenter, adiRadius, opts, pxPerDeg, pal);

    // 5. Radio Altimeter AGL Readout near bottom of ADI sphere (when < 2500 ft)
    if (opts.radioAltitudeFeet < 2500.0f) {
        const keel::f32 raY = adiRect.Bottom() - (opts.showIlsDiamonds && lay.isFullPfd ? 34.0f : 20.0f);
        const Rect raBox(adiCenter.x - 36.0f, raY - 9.0f, 72.0f, 18.0f);
        canvas.DrawRoundRect(raBox, 3.0f, Color(6, 9, 16, 215), pal.tapeBorder, 1.0f);
        char raBuf[24];
        const Color raCol = (opts.radioAltitudeFeet < 400.0f) ? pal.amberWarn : pal.fmaGreen;
        canvas.DrawTextBox(keel::format_buf(raBuf, "RA %4.0f", keel::max(0.0f, opts.radioAltitudeFeet)), raBox, 10.5f, raCol, 0.5f, 0.5f);
    }

    // 6. Central Aircraft Reference Symbol
    DrawAircraftReferenceSymbol(canvas, adiCenter, adiRadius, opts, pal);

    // 7. Interactive Mouse / Virtual Stick Vector Overlay
    if (opts.showMouseStickOverlay && resp.cursorInsideAdi) {
        const Vec2 stickPt{
            adiCenter.x + resp.cursorNorm.x * (adiRadius * 0.78f),
            adiCenter.y + resp.cursorNorm.y * (adiRadius * 0.78f)
        };
        canvas.DrawLine(adiCenter, stickPt, Color(56, 189, 248, 145), 1.4f);
        DrawCircleOutline(canvas, stickPt, 7.0f, Color(56, 189, 248, 235), 1.6f, 20);
        DrawFilledCircle(canvas, stickPt, 2.2f, Color(250, 204, 21, 245));
    }
    canvas.PopClip();

    // 8. Outer ADI Bezel Frame (Circular ring in Circular mode, Rounded dome frame in RoundedDome mode)
    if (lay.isCircular) {
        DrawCircleOutline(canvas, adiCenter, adiRadius, pal.bezelRing, 4.0f, 64);
        DrawCircleOutline(canvas, adiCenter, adiRadius + 1.5f, pal.bezelHighlight, 1.2f, 64);
    } else if (!lay.isFullBleed) {
        const keel::f32 domeRadius = adiRect.w * 0.22f;
        canvas.DrawRoundRect(adiRect, domeRadius, Color::Transparent(), pal.bezelHighlight, 1.5f);
    }

    // 9. ILS Localizer & Glideslope Deviation Diamonds
    if (opts.showIlsDiamonds && !lay.isCompact) {
        DrawIlsDeviationScales(canvas, adiRect, opts, pal);
    }

    // 10. Top FMA Strip, Left Airspeed Tape, Right Altitude/VSI Tape, Bottom Heading Tape
    if (lay.fmaH > 0.0f) {
        const Rect fmaRect(r.x + lay.pad, r.y + lay.pad, r.w - lay.pad * 2.0f, lay.fmaH);
        DrawFmaBar(canvas, fmaRect, opts, pal);
    }

    if (lay.leftTapeW > 0.0f) {
        const Rect iasRect(r.x + lay.pad, lay.availCenterRect.y, lay.leftTapeW, lay.availCenterRect.h);
        DrawAirspeedTape(canvas, iasRect, opts, pal);
    }

    if (lay.rightTapeW > 0.0f) {
        const Rect altRect(r.Right() - lay.pad - lay.rightTapeW, lay.availCenterRect.y, lay.rightTapeW, lay.availCenterRect.h);
        DrawAltitudeAndVsiTape(canvas, altRect, opts, pal);
    }

    if (lay.hdgH > 0.0f) {
        const Rect hdgRect(adiRect.x, r.Bottom() - lay.pad - lay.hdgH, adiRect.w, lay.hdgH);
        DrawHeadingTape(canvas, hdgRect, opts, pal);
    }

    // 11. Corner Digital Telemetry Readouts (in CompactADI or HUD mode when side tapes are hidden)
    if (opts.showSpeedAndAltTapes && lay.leftTapeW <= 0.0f) {
        const keel::f32 topTagY = lay.availCenterRect.y + 2.0f;
        char tlBuf[32];
        const keel::StringRef tlRef = keel::format_buf(tlBuf, "IAS %3.0f KT", opts.airspeedKnots);
        const keel::f32 tlW = canvas.MeasureText(tlRef, 11.0f).x;
        if (lay.isFullBleed && !lay.isHud) {
            const Rect tlPill(r.x + 8.0f, topTagY - 2.0f, tlW + 12.0f, 20.0f);
            canvas.DrawRoundRect(tlPill, 3.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
            canvas.DrawTextBox(tlRef, tlPill, 11.0f, pal.fmaGreen, 0.5f, 0.5f);
        } else {
            canvas.DrawText(tlRef, {r.x + 12.0f, topTagY}, 11.0f, pal.fmaGreen);
        }

        char trBuf[32];
        const keel::StringRef trRef = keel::format_buf(trBuf, "ALT %5.0f FT", opts.altitudeFeet);
        const keel::f32 trW = canvas.MeasureText(trRef, 11.0f).x;
        if (lay.isFullBleed && !lay.isHud) {
            const Rect trPill(r.Right() - trW - 20.0f, topTagY - 2.0f, trW + 12.0f, 20.0f);
            canvas.DrawRoundRect(trPill, 3.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
            canvas.DrawTextBox(trRef, trPill, 11.0f, pal.fmaCyan, 0.5f, 0.5f);
        } else {
            canvas.DrawText(trRef, {r.Right() - trW - 12.0f, topTagY}, 11.0f, pal.fmaCyan);
        }
    }

    // Bottom corner attitude & G-load summary tags
    {
        char blBuf[48];
        const keel::StringRef blRef = keel::format_buf(blBuf, "BNK %+05.1f°  PIT %+05.1f°", opts.rollDeg, opts.pitchDeg);
        const keel::f32 blW = canvas.MeasureText(blRef, 10.0f).x;

        char brBuf[48];
        const keel::StringRef brRef = keel::format_buf(brBuf, "HDG %03.0f°  G %+.1f", NormalizeHeadingDeg(opts.headingDeg), opts.gLoad);
        const keel::f32 brW = canvas.MeasureText(brRef, 10.0f).x;

        if (lay.isFullBleed && !lay.isHud) {
            const keel::f32 pillH = 20.0f;
            const keel::f32 pillY = ((lay.hdgH > 0.0f) && (adiRect.x < (r.x + lay.pad + blW + 18.0f)))
                ? (r.Bottom() - lay.pad - lay.hdgH - 6.0f - pillH)
                : (r.Bottom() - lay.pad - pillH);
            const Rect blPill(r.x + lay.pad, pillY, blW + 12.0f, pillH);
            const Rect brPill(r.Right() - lay.pad - brW - 12.0f, pillY, brW + 12.0f, pillH);
            canvas.DrawRoundRect(blPill, 3.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
            canvas.DrawTextBox(blRef, blPill, 10.0f, pal.tapeTick, 0.5f, 0.5f);
            canvas.DrawRoundRect(brPill, 3.0f, pal.tapeBg, pal.tapeBorder, 1.0f);
            canvas.DrawTextBox(brRef, brPill, 10.0f, pal.tapeTick, 0.5f, 0.5f);
        } else {
            canvas.DrawText(blRef, {r.x + 12.0f, r.Bottom() - 18.0f}, 10.0f, pal.tapeTick);
            canvas.DrawText(brRef, {r.Right() - brW - 12.0f, r.Bottom() - 18.0f}, 10.0f, pal.tapeTick);
        }
    }

    if (lay.isFullBleed && borderW > 0.0f) {
        canvas.DrawRoundRect(r, radius, Color::Transparent(), borderCol, borderW);
    }

    canvas.PopClip();
}

} // namespace

Vec2 AttitudeView::Measure(
    const AttitudeViewOptions& opts,
    UiSize /*size*/,
    ICanvas* /*canvas*/,
    const UiTheme& /*theme*/,
    keel::f32 availableWidth
) {
    keel::f32 defaultW = 520.0f;
    if (opts.width.IsFixed()) {
        defaultW = opts.width.Dp();
    } else if (opts.width.ResolveMode(SizeMode::Stretch) == SizeMode::Stretch && availableWidth > 0.0f) {
        defaultW = availableWidth;
    }
    const keel::f32 defaultH = opts.height.IsFixed()
        ? opts.height.Dp()
        : 420.0f;
    return {defaultW, defaultH};
}

AttitudeViewResponse AttitudeView::Run(
    Context& ctx,
    const AttitudeViewOptions& opts,
    keel::SourceLocation loc
) {

    const Id id = ctx.MakeChildId(loc);
    const bool isDisabled = opts.disabled || ctx.IsDisabled();
    const UiSize effectiveSize = opts.size;

    Response baseRes = ctx.Interact(id);
    AttitudeViewResponse resp{};
    static_cast<Response&>(resp) = baseRes;
    resp.displayMode = opts.displayMode;

    Rect prevBounds = baseRes.rect;
    if (prevBounds.w <= 1.0f || prevBounds.h <= 1.0f) {
        prevBounds = ctx.GetParentPrevFrameBounds();
    }
    const Rect activeRect = prevBounds;

    auto& state = ctx.GetOrCreatePersistentState<AttitudeViewState>(id);
    if (!state.initialized) {
        state.fovPitchDeg = keel::clamp(opts.fieldOfViewPitchDeg, 25.0f, 90.0f);
        state.initialized = true;
    }
    if (opts.fovPitchDegPtr) {
        state.fovPitchDeg = keel::clamp(*opts.fovPitchDegPtr, 25.0f, 90.0f);
    } else {
        state.fovPitchDeg = keel::clamp(opts.fieldOfViewPitchDeg, 25.0f, 90.0f);
    }

    // Resolve ADI sub-rectangle for accurate mouse-to-attitude mapping
    const AttitudeLayout lay = ComputeAttitudeLayout(activeRect, opts);
    resp.viewportMode = lay.viewportMode;
    resp.horizonRect = lay.horizonRect;
    resp.adiRect = lay.adiRect;
    resp.adiCenter = lay.adiCenter;
    resp.adiRadius = lay.adiRadius;

    const Vec2 mousePos = ctx.GetMousePos();
    const bool hovered = !isDisabled && ctx.IsRectHovered(id, activeRect);
    resp.hovered = hovered;
    resp.cursorInside = hovered;

    if (hovered) {
        const Vec2 wheel = ctx.GetMouseWheel();
        if (keel::abs(wheel.y) > 0.01f) {
            state.fovPitchDeg = keel::clamp(state.fovPitchDeg - wheel.y * 4.0f, 25.0f, 90.0f);
            if (opts.fovPitchDegPtr) {
                *opts.fovPitchDegPtr = state.fovPitchDeg;
            }
            ctx.ConsumeMouseWheel();
            ctx.RequestRepaint(2);
            resp.changed = true;
        }

        const keel::f32 invR = (resp.adiRadius > 1.0f) ? (1.0f / resp.adiRadius) : 0.0f;
        const keel::f32 nx = keel::clamp((mousePos.x - resp.adiCenter.x) * invR, -1.0f, 1.0f);
        const keel::f32 ny = keel::clamp((mousePos.y - resp.adiCenter.y) * invR, -1.0f, 1.0f);
        resp.cursorNorm = {nx, ny};
        resp.cursorInsideAdi = lay.horizonRect.Contains(mousePos);
        resp.cursorRollDeg = nx * 65.0f;
        resp.cursorPitchDeg = -ny * 40.0f;
    }

    resp.visibleFovPitchDeg = state.fovPitchDeg;

    if (hovered && !opts.tooltip.empty()) {
        ctx.SetTooltip(opts.tooltip, baseRes.rect);
    }

    Node* node = ctx.CreateNode(keel::from_underlying<NodeType>(keel::as<keel::u16>(NodeType::User) + 10u), id);
    auto* payload = ctx.AllocPayload<AttitudeViewPayload>();
    payload->opts = opts;
    payload->opts.tooltip = ctx.AllocText(opts.tooltip);
    payload->resp = resp;

    node->text = "AttitudeView";
    node->SetPayload(payload);

    node->visualHash = detail::HashArgs(
        opts.background,
        opts.border,
        keel::as<keel::u8>(opts.displayMode),
        keel::as<keel::u8>(opts.viewportMode),
        keel::as<keel::u8>(opts.referenceMode),
        keel::as<keel::u8>(opts.colorScheme),
        keel::as<keel::u8>(opts.flightDirectorStyle),
        keel::as<keel::u8>(opts.aircraftSymbol),
        opts.rollDeg,
        opts.pitchDeg,
        opts.headingDeg,
        opts.airspeedKnots,
        opts.machNumber,
        opts.altitudeFeet,
        opts.verticalSpeedFpm,
        opts.slipSkid,
        opts.gLoad,
        opts.radioAltitudeFeet,
        opts.showFlightPathVector,
        opts.aoaDeg,
        opts.sideslipDeg,
        opts.fdPitchDeg,
        opts.fdRollDeg,
        opts.targetAirspeedBug,
        opts.targetAltitudeBug,
        opts.targetHeadingBug,
        opts.showIlsDiamonds,
        opts.locDeviationDots,
        opts.gsDeviationDots,
        opts.showPitchLadder,
        opts.showBankScale,
        opts.showSpeedAndAltTapes,
        opts.showHeadingTape,
        opts.showFmaBar,
        opts.showMouseStickOverlay,
        state.fovPitchDeg,
        resp.cursorInsideAdi ? resp.cursorNorm.x : -2.0f,
        resp.cursorInsideAdi ? resp.cursorNorm.y : -2.0f,
        activeRect.w,
        activeRect.h,
        effectiveSize,
        isDisabled
    );

    node->measureFn = [](const Node* n, keel::f32 availableWidth, ICanvas* c, const UiTheme& th) noexcept -> Vec2 {
        const auto* p = n->As<AttitudeViewPayload>();
        if (!p) return {0.0f, 0.0f};
        AttitudeViewOptions effOpts = p->opts;
        if (n->widthMode == SizeMode::Stretch && !effOpts.width.IsFixed()) {
            effOpts.width = SizeMode::Stretch;
        }
        return AttitudeView::Measure(effOpts, n->uiSize, c, th, availableWidth);
    };

    node->renderFn = &RenderAttitudeViewNode;

    node->uiSize = effectiveSize;
    node->disabled = isDisabled;
    node->hovered = hovered;
    node->pressed = baseRes.pressed;

    ControlSpec ctrlSpec{};
    ctrlSpec.width = opts.width;
    ctrlSpec.height = opts.height;
    ctrlSpec.defaultWidthMode = SizeMode::Stretch;
    ctrlSpec.defaultHeightMode = SizeMode::Stretch;
    ctrlSpec.minHeight = opts.height.IsFixed() ? opts.height.Dp() : 260.0f;
    Control::SetupNode(ctx, node, ctrlSpec);

    return resp;
}

} // namespace guikit
