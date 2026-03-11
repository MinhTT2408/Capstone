# Pattern 3: Cascading Half-Cycle Sequential Motion

## Overview
A complex sequential pattern where all motors perform sine wave movements in half-cycle segments, with hold states between movements. Motors take turns in a cascading fashion.

## Timing Diagram

```
Time Step | Motor 1        | Motor 2        | Motor 3        | Duration
----------|----------------|----------------|----------------|----------
t=0       | ----           | ----           | ----           | 0ms
t=1       | 0→Half ^^^^    | ----           | ----           | 2500ms
t=2       | HOLD ====      | 0→Half ^^^^    | ----           | 2500ms
t=3       | RST→Half ^^^^  | HOLD ====      | 0→Half ^^^^    | 2500ms
t=4       | HOLD ====      | RST→Half ^^^^  | HOLD ====      | 2500ms
t=5       | RST→Half ^^^^  | HOLD ====      | RST→Half ^^^^  | 2500ms
[Repeats from t=2]

Legend:
  ----  : Idle (motor off)
  ^^^^  : Moving (executing sine half-cycle from 0 to peak)
  ====  : Hold (maintaining position with PID)
  RST   : Position reset to zero before next movement
```

## Position Graph (Amplitude = 1 revolution = 8500 counts)

```
Position
(counts)
  8500 │                    ╭────╮         ╭────╮       
       │         ╭────╮    │     │        │     │       Motor 1: ━━━━
  6000 │        │     │   │      │       │      │      Motor 2: ┄┄┄┄
       │       │      │  │       │      │       │      Motor 3: ─ ─ ─
  4000 │      │       │ │        │     │        │     
       │     │        ││         │    │         │    
  2000 │    │         ││         │   │          │   
       │   │          ││         │  │           │  
     0 └───┴──────────┴┴─────────┴──┴───────────┴───> Time
       0   t1   t2   t3   t4   t5  [repeat cycle]

Motor 1 (━━━━): 
  t1: Ramps 0→8500
  t2: Holds at 8500
  t3: Resets to 0, ramps 0→8500
  t4: Holds at 8500
  t5: Resets to 0, ramps 0→8500

Motor 2 (┄┄┄┄):
  t1: Idle at 0
  t2: Ramps 0→8500
  t3: Holds at 8500
  t4: Resets to 0, ramps 0→8500
  t5: Holds at 8500

Motor 3 (─ ─ ─):
  t1-t2: Idle at 0
  t3: Ramps 0→8500
  t4: Holds at 8500
  t5: Resets to 0, ramps 0→8500
```

## Detailed State Machine

### Time Step 1 (0-2.5s)
- **Motor 1**: MOVING (0→Peak) - Executes first half of sine wave
- **Motor 2**: IDLE - Not started
- **Motor 3**: IDLE - Not started

### Time Step 2 (2.5-5.0s)
- **Motor 1**: HOLD at Peak - Maintains position with active PID
- **Motor 2**: MOVING (0→Peak) - Begins its first half-cycle
- **Motor 3**: IDLE - Not started

### Time Step 3 (5.0-7.5s)
- **Motor 1**: MOVING (0→Peak) - Position reset, executes new half-cycle
- **Motor 2**: HOLD at Peak - Maintains position
- **Motor 3**: MOVING (0→Peak) - Begins its first half-cycle

### Time Step 4 (7.5-10.0s)
- **Motor 1**: HOLD at Peak - Maintains position
- **Motor 2**: MOVING (0→Peak) - Position reset, executes new half-cycle
- **Motor 3**: HOLD at Peak - Maintains position

### Time Step 5 (10.0-12.5s)
- **Motor 1**: MOVING (0→Peak) - Position reset, executes new half-cycle
- **Motor 2**: HOLD at Peak - Maintains position
- **Motor 3**: MOVING (0→Peak) - Position reset, executes new half-cycle

**Pattern then repeats from Time Step 2**

## Key Features

1. **Half-Cycle Duration**: Each movement segment is `SINE_DURATION_MS / 2`
2. **Full Pattern Period**: 5 × (SINE_DURATION_MS / 2) = 12.5 seconds (at default 5s sine duration)
3. **Hold State**: Motors maintain their peak position using active PID control
4. **Position Reset**: Before each new movement phase, encoder is reset to 0
5. **Cascading Effect**: Creates a wave-like motion across the three motors

## Implementation Notes

- Uses state machine with states: `IDLE`, `MOVING_FIRST_HALF`, `HOLD_AT_HALF`, `MOVING_SECOND_HALF`, `HOLD_AT_FULL`
- Each motor tracks its own state independently
- PID continuously active during both MOVING and HOLD states
- Position resets allow motors to repeat half-cycles without accumulating position

## Use Cases

- Synchronized mechanical systems requiring sequential activation
- Wave propagation demonstrations
- Multi-actuator systems with position-hold requirements
- Testing motor coordination and PID hold stability
