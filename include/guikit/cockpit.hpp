#pragma once

// ==============================================================================
// GuiKit Cockpit Module - Avionics PFD & Automotive Instrument Cluster Views
// ==============================================================================

#include "guikit/cockpit/attitude_view.hpp"
#include "guikit/cockpit/car_cluster_view.hpp"
#include "guikit/context.hpp"

namespace guikit::cockpit {

/// Renders an Avionics Artificial Horizon & Primary Flight Display (PFD) widget.
inline AttitudeViewResponse AttitudeView(
    Context& ui,
    const AttitudeViewOptions& opts = {},
    keel::SourceLocation loc = keel::SourceLocation::current()
) {
    WidgetExecGuard guard(ui);
    return guikit::AttitudeView::Run(ui, opts, loc);
}

/// Renders an Automotive Digital Instrument Cluster & ADAS Cockpit widget.
inline CarClusterViewResponse CarClusterView(
    Context& ui,
    const CarClusterViewOptions& opts = {},
    keel::SourceLocation loc = keel::SourceLocation::current()
) {
    WidgetExecGuard guard(ui);
    return guikit::CarClusterView::Run(ui, opts, loc);
}

} // namespace guikit::cockpit
