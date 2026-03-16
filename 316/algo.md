# 316 Algorithm Notes

## Current behavior

- Sensor logic is normalized to `black=1`, `white=0`.
- Target station is selected with the single `START` button on `D6`.
- Short press cycles `A -> B -> C -> A`.
- Long press starts the run only when the robot is on `0110`.

## Marker handling

- `1111` is the station mark.
- `0000` first becomes a `gap candidate`, not an immediate gap count.
- If the current station has already been handled and the robot is still physically on `1111`, the drive logic keeps pushing forward instead of stopping on the black bar.
- If the white region ends before `END_CONFIRM_MS`, it is resolved as a normal gap.
- If the white region lasts longer than `END_CONFIRM_MS`, the code checks whether a turnaround is actually allowed before treating it as the line end.

## Turnaround gating

- The sketch tracks the last confirmed station and inferred travel direction.
- Direction is inferred from station order:
  - first confirmed `A` implies `TO_C`
  - first confirmed `C` implies `TO_A`
  - later increasing station order keeps/sets `TO_C`
  - later decreasing station order keeps/sets `TO_A`
- A long white region can trigger turnaround only when:
  - direction is `TO_A` and the last confirmed station is `A`, or
  - direction is `TO_C` and the last confirmed station is `C`
- This prevents ordinary off-line white regions from being misread as the terminal end.

## Target drop policy

- The target station is served at most once per leg.
- After one successful drop, later sightings of the same target station in the same leg are forced to `pass`.
- The flag resets only after a confirmed turnaround.

## Debug output

- Runtime status includes:
  - `mode`
  - `box`
  - `pass`
  - `last`
  - `dir`
  - `arm`
  - `gapMs` while a white-region candidate is active
- Important events:
  - `[TURN] canceled by station`
  - `[ST] station detected`
  - `[GAP] candidate`
- `[END] confirmed ...`
- `[END] blocked ...`
- `[GAP] long candidate -> gap ...`

## Tuning points

- Motion is tuned through the servo angle constants at the top of `316.ino`.
- White-region / debounce timing is mainly controlled by:
  - `TURN_SIGNAL_CONFIRM_MS`
  - `END_CONFIRM_MS`
  - `GAP_EVENT_COOLDOWN_MS`
  - `STATION_EVENT_COOLDOWN_MS`
  - `STATUS_PRINT_MS`
