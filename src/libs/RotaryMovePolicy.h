#pragma once

namespace rotary_move {

enum class XYOrder {
  together,
  x_then_y,
  y_then_x,
};

constexpr bool enabled(bool configured, bool rotary_fitted) { return configured && rotary_fitted; }

constexpr XYOrder clearance_order(bool split) { return split ? XYOrder::y_then_x : XYOrder::together; }

constexpr XYOrder destination_order(bool split) { return split ? XYOrder::x_then_y : XYOrder::together; }

}  // namespace rotary_move
