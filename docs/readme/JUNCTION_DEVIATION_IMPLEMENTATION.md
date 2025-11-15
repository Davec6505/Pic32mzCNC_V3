# Junction Deviation Implementation Plan
## November 10, 2025

### Overview
Add GRBL-style junction deviation for smooth cornering between motion segments. This enables continuous motion through corners at safe speeds instead of stopping between each segment.

### Current State Analysis
✅ **Ready for Implementation:**
- Trapezoidal velocity profiling complete
- Multi-segment motion queue (16 segments)
- Clean kinematics interface with start/end velocity fields
- Robust segment loading and execution system

### Phase 1: Basic Junction Deviation (4-6 hours)

#### 1.1 Add GRBL Setting ($11)
**File:** `incs/settings/settings.h`
```c
// Add after line 58 (homing_pull_off):
float junction_deviation;       // $11 - Junction deviation (mm)
```

**File:** `srcs/settings/settings.c`
```c
// Add to default settings (line ~76):
.junction_deviation = 0.01f,    // GRBL default: 0.01mm

// Add to SETTINGS_SetValue() switch (around line 225):
case 11: settings->junction_deviation = value; break;

// Add to SETTINGS_GetValue() switch (around line 290):
case 11: return settings->junction_deviation;
```

#### 1.2 Junction Speed Calculation
**File:** `srcs/motion/kinematics.c`
```c
// Add new function:
float KINEMATICS_CalculateJunctionSpeed(CoordinatePoint prev_dir, 
                                       CoordinatePoint curr_dir,
                                       float junction_deviation,
                                       float acceleration) {
    // Calculate angle between direction vectors
    float dot_product = prev_dir.x * curr_dir.x + 
                       prev_dir.y * curr_dir.y + 
                       prev_dir.z * curr_dir.z;
    
    // Clamp to [-1, 1] to prevent acos domain errors
    if (dot_product > 1.0f) dot_product = 1.0f;
    if (dot_product < -1.0f) dot_product = -1.0f;
    
    // Straight line (no junction needed)
    if (dot_product > 0.999f) {
        return 1000.0f;  // High speed (no corner)
    }
    
    // Sharp corner calculation using GRBL formula
    float cosine_angle = dot_product;
    if (cosine_angle < -0.999f) {
        return 0.0f;  // Dead stop for reverse direction
    }
    
    // GRBL junction deviation formula
    float junction_speed = sqrtf(junction_deviation * acceleration * 
                                (2.0f / (1.0f - cosine_angle)));
    
    return junction_speed;
}
```

#### 1.3 Modify Linear Move Planning
**File:** `srcs/motion/kinematics.c` - Update `KINEMATICS_LinearMove()`

Add parameters to enable junction planning:
```c
// Update function signature:
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, 
                                   float feedrate, MotionSegment* segment_buffer,
                                   float entry_velocity,    // NEW: velocity entering this segment
                                   float exit_velocity);    // NEW: velocity exiting this segment

// Update final_rate calculation (around line 198):
// OLD: segment_buffer->final_rate = segment_buffer->initial_rate;
// NEW: 
float min_steps_per_sec = 500.0f;
float exit_steps_per_sec = fmaxf(exit_velocity * steps_per_mm_dominant, min_steps_per_sec);
segment_buffer->final_rate = (uint32_t)(TIMER_FREQ / exit_steps_per_sec);

float entry_steps_per_sec = fmaxf(entry_velocity * steps_per_mm_dominant, min_steps_per_sec);
segment_buffer->initial_rate = (uint32_t)(TIMER_FREQ / entry_steps_per_sec);
```

#### 1.4 Two-Segment Lookahead
**File:** `srcs/motion/motion.c` - Update `MOTION_ProcessGcodeEvent()`

Add junction planning logic:
```c
// After line 420 (before adding to queue):
// Check if we can apply junction deviation to previous segment
if (appData->motionQueueCount > 0) {
    // Get previous segment
    uint32_t prev_index = (appData->motionQueueHead - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
    MotionSegment* prev_segment = &appData->motionQueue[prev_index];
    
    // Calculate direction vectors (normalized)
    CoordinatePoint prev_dir = {
        .x = (float)prev_segment->delta_x / (float)prev_segment->dominant_delta,
        .y = (float)prev_segment->delta_y / (float)prev_segment->dominant_delta,
        .z = (float)prev_segment->delta_z / (float)prev_segment->dominant_delta,
        .a = (float)prev_segment->delta_a / (float)prev_segment->dominant_delta
    };
    
    CoordinatePoint curr_dir = {
        .x = (float)segment->delta_x / (float)segment->dominant_delta,
        .y = (float)segment->delta_y / (float)segment->dominant_delta,
        .z = (float)segment->delta_z / (float)segment->dominant_delta,
        .a = (float)segment->delta_a / (float)segment->dominant_delta
    };
    
    // Get settings for junction calculation
    CNC_Settings* settings = SETTINGS_GetCurrent();
    float junction_speed = KINEMATICS_CalculateJunctionSpeed(prev_dir, curr_dir, 
                                                            settings->junction_deviation,
                                                            *(axis_cfg->acceleration));
    
    // Update previous segment's exit velocity
    if (junction_speed > 0.0f && junction_speed < prev_segment->max_velocity) {
        // Recalculate previous segment with new exit velocity
        // This may require re-planning its deceleration phase
    }
}
```

### Phase 2: Full Multi-Segment Planner (Future)
- Look-ahead across multiple segments (3-8 segments)
- Velocity optimization across entire planned path
- Segment merging for co-linear moves

### Implementation Timeline
**Day 1:** Settings and basic junction speed calculation (2 hours)
**Day 2:** Integrate junction planning into kinematics (2-3 hours)  
**Day 3:** Two-segment lookahead and testing (2-3 hours)

### Testing Strategy
```gcode
# Test 1: Sharp 90-degree corner
G1 X10 Y0 F1000
G1 X10 Y10 F1000

# Test 2: Gentle corner  
G1 X10 Y0 F1000
G1 X15 Y5 F1000

# Test 3: Complex path
G1 X10 Y0 F1000
G1 X10 Y10 F1000
G1 X0 Y10 F1000
G1 X0 Y0 F1000
```

**Expected Results:**
- Sharp corners: Significant deceleration, smooth transition
- Gentle corners: Minimal deceleration, faster overall
- Complex paths: Continuous motion, no stops between segments

### Risk Mitigation
1. **Backward Compatibility**: Default `junction_deviation = 0.01` provides smooth motion
2. **Fallback Mode**: Can disable with `$11=0` for exact-stop behavior
3. **Incremental Implementation**: Start with 2-segment lookahead, expand gradually
4. **Validation**: Existing single-segment motion unchanged

### Benefits
- **Smoother Motion**: Eliminates stops between segments
- **Faster Machining**: Higher overall feed rates through optimized cornering
- **Better Surface Finish**: Continuous motion reduces tool marks
- **GRBL Compatibility**: Standard $11 setting matches industry expectations