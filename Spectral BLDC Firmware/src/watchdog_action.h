// watchdog_action.h -- what the driver does when the CAN watchdog trips.
//
// Pure decision logic, deliberately free of hardware includes so it can be unit-tested
// on a host compiler. Identical file in STEPFOC and Spectral.
//
// Wire byte (IN_WATCHDOG_TIMEOUT data[4]) -- matches Spectral_BLDC.Send_Watchdog:
//   0 'Idle'    legacy: whatever THIS firmware always did on a watchdog trip
//               (STEPFOC: brake per controller.brake_coast; Spectral: coast, driver off).
//               RTI and the lib's default send 0, so an unmodified host is unchanged.
//   1 'Hold'    keep holding the position the joint had when the watchdog tripped, for the
//               fallback time, then the after-action.
//   2 'Brake'   short the phases.
//   3 'Coast'   phases Hi-Z.
//   4 'Descend' apply the fallback current, but never let the joint move faster than the
//               fallback speed in either direction, for the fallback time, then the
//               after-action. With 0 mA a joint under gravity lowers itself at no more than
//               that speed and rests on whatever stops it; with a gripping current a gripper
//               keeps its grip and its jaws cannot slam shut.
// Anything else is treated as legacy.
//
// Hold and descend only start if the drive was running a closed-loop mode when the watchdog
// tripped; a drive that was idle stays as idle was (legacy). They are always bounded by the
// fallback time (parameter PARAM_WD_FALLBACK_TIME), so a dead host can never leave a motor driven forever.
//
// The action applies ONLY when the watchdog is the sole active error. Any other fault
// (over-voltage, encoder, e-stop, not calibrated, ...) always gets legacy behaviour: a
// gentler response chosen for a lost host must never soften the response to a real fault.
#pragma once

#define WD_ACTION_LEGACY 0
#define WD_ACTION_HOLD 1
#define WD_ACTION_BRAKE 2
#define WD_ACTION_COAST 3
#define WD_ACTION_DESCEND 4

enum WdResponse
{
    WD_RESP_LEGACY = 0, // do what the firmware always did
    WD_RESP_BRAKE = 1,
    WD_RESP_COAST = 2,
    WD_RESP_HOLD = 3,
    WD_RESP_DESCEND = 4
};

/// @param action           the stored watchdog action byte
/// @param only_watchdog    the watchdog is the only active error
/// @param fallback_active  hold/descend was started at the trip (the drive was in closed loop)
/// @param fallback_expired the fallback time has run out
/// @param after_action     what follows hold/descend: WD_ACTION_BRAKE, WD_ACTION_COAST, else legacy
static inline WdResponse watchdog_response(int action, bool only_watchdog, bool fallback_active,
                                           bool fallback_expired, int after_action)
{
    if (!only_watchdog)
        return WD_RESP_LEGACY;
    switch (action)
    {
    case WD_ACTION_BRAKE:
        return WD_RESP_BRAKE;
    case WD_ACTION_COAST:
        return WD_RESP_COAST;
    case WD_ACTION_HOLD:
    case WD_ACTION_DESCEND:
        if (!fallback_active)
            return WD_RESP_LEGACY; // tripped while idle: stay as idle was
        if (!fallback_expired)
            return (action == WD_ACTION_HOLD) ? WD_RESP_HOLD : WD_RESP_DESCEND;
        if (after_action == WD_ACTION_BRAKE)
            return WD_RESP_BRAKE;
        if (after_action == WD_ACTION_COAST)
            return WD_RESP_COAST;
        return WD_RESP_LEGACY;
    default: // 0 (legacy), anything unknown
        return WD_RESP_LEGACY;
    }
}
