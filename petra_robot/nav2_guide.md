# Adaptive Monte Carlo Localization (AMCL)

**AMCL** (Adaptive Monte Carlo Localization) is a method for helping a robot figure out where it is on a known map. It uses a particle filter to compare laser scan data against the map, refining the robot’s estimated position as it moves around.

### **Analogy: Blindfold in a Familiar Room**
Imagine you’re blindfolded in your house. You touch walls or furniture, matching what you “sense” against your mental map, until you become more confident about where you stand. AMCL works similarly: it takes a bunch of “guesses” (particles) for the robot’s position, compares sensor data to the map, and then narrows down which guesses are most likely correct.

### **How AMCL Works**
1. **Particle Distribution**: A large number of hypothetical positions (particles) are placed across the map.
2. **Sensor Updates**: The robot uses its LIDAR sensor to:
   - Measure the environment (walls, corners, etc.).
   - Compare these measurements to what the map would predict at each particle’s position.
3. **Scoring & Resampling**:
   - Particles that match the real sensor readings better get a higher score.
   - Low-scoring particles get discarded, while new ones get spawned around high-scoring guesses.
4. **Adaptive Behavior**:
   - Uses more particles when the robot is very uncertain about its position (like at startup).
   - Reduces the number of particles once the position is estimated with high confidence.

### **Why it’s Important**
If the robot thinks it’s in the wrong place, all navigation decisions (like path planning) become unreliable. AMCL’s job is to provide a highly accurate pose estimate so that every other navigation component can do its job effectively.

---

## **AMCL Parameters**

Below is a list of common AMCL parameters, along with brief explanations of what they do, default values, and typical usage notes.

### **Odometry Noise Parameters**  
These `alpha` parameters tell AMCL how uncertain (noisy) to consider the robot’s odometry information:

1. **alpha1** *(unitless)*  
   - **Default**: 0.2  
   - **Purpose**: Affects rotation estimate from rotation. A higher value means AMCL trusts the robot’s rotation less and relies more on sensor data.

2. **alpha2** *(unitless)*  
   - **Default**: 0.2  
   - **Purpose**: Affects rotation estimate from translation (when the robot moves straight). Higher means more uncertainty in heading while moving forward.

3. **alpha3** *(unitless)*  
   - **Default**: 0.2  
   - **Purpose**: Affects position estimates from forward motion. Higher means the robot trusts its wheel encoders less when moving forward.

4. **alpha4** *(unitless)*  
   - **Default**: 0.2  
   - **Purpose**: Affects position estimates from rotation. Higher values mean more caution about position changes during turns.

5. **alpha5** *(unitless)*  
   - **Default**: 0.2  
   - **Purpose**: Applies to omnidirectional robots for lateral (sideways) movement. Higher value indicates more uncertainty in sideways motion.

---

### **Frame IDs**
- **base_frame_id** *(string)*  
  - **Default**: `"base_footprint"`  
  - The name of the robot’s base frame.

- **global_frame_id** *(string)*  
  - **Default**: `"map"`  
  - The name of the map (or “global”) frame in your TF tree.

- **odom_frame_id** *(string)*  
  - **Default**: `"odom"`  
  - The name of the odometry frame used for tracking the robot’s movement.

---

### **Particle Filter Details**
- **max_particles** *(unitless)*  
  - **Default**: 2000  
  - The maximum number of particles. Increasing boosts accuracy (but raises CPU use).

- **min_particles** *(unitless)*  
  - **Default**: 500  
  - The minimum number of particles. Increasing ensures wider coverage but uses more computing power.

- **pf_err** *(unitless)*  
  - **Default**: 0.05  
  - Controls how aggressively the filter reduces the particle count to maintain performance.

- **pf_z** *(unitless)*  
  - **Default**: 0.99  
  - Influences how quickly particles are removed. Higher means more particles stick around longer.

- **recovery_alpha_fast** *(unitless)*  
  - **Default**: 0.0  
  - Allows quick recovery if the robot is lost. If it’s above 0, fast recovery will spread particles broadly.

- **recovery_alpha_slow** *(unitless)*  
  - **Default**: 0.0  
  - Allows a slower, more methodical particle spread for recovery. Also above 0 to enable.

- **resample_interval** *(unitless)*  
  - **Default**: 1  
  - Resamples the particle filter after every filter update by default. Increasing means less frequent resampling.

---

### **Laser Model Parameters**
These relate to how AMCL interprets LIDAR (laser scanner) data to score particles:

- **laser_model_type** *(string)*  
  - **Default**: `"likelihood_field"`  
  - Choices include:
    1. `"likelihood_field"` (fast, default)  
    2. `"beam"` (more accurate but slower)  
    3. `"likelihood_field_prob"` (adds probabilistic calculations, middle ground)

- **max_beams** *(unitless)*  
  - **Default**: 60  
  - The maximum number of beams taken from each laser scan to update the filter.

- **laser_min_range** *(meters)*  
  - **Default**: -1.0 (use sensor’s min range)  
  - Increase if your sensor has unreliable very-close readings.

- **laser_max_range** *(meters)*  
  - **Default**: 100.0 (or `-1.0` to use sensor’s max)  
  - Decrease if you only want to trust nearer readings for faster computation.

- **laser_likelihood_max_dist** *(meters)*  
  - **Default**: 2.0  
  - The maximum distance within which obstacles are considered in the likelihood field model.

- **beam_skip_* parameters**  
  1. **do_beamskip** *(bool)* – Default: `False`. If `true`, can skip “bad” beams that don’t match the map.  
  2. **beam_skip_distance** *(meters)* – Default: `0.5`. Threshold for how far off a beam must be before skipping.  
  3. **beam_skip_threshold** *(unitless)* – Default: `0.3`. Fraction of beams allowed to be skipped.  
  4. **beam_skip_error_threshold** *(unitless)* – Default: `0.9`. If too many beams are skipped, trigger a full update.

---

### **Probability Weights**
These parameters define how sensor readings (LIDAR) are weighted within the probability model:

1. **z_hit** *(unitless)*  
   - **Default**: 0.5  
   - Weight for sensor readings that perfectly match the map.

2. **z_max** *(unitless)*  
   - **Default**: 0.05  
   - Weight for max-range readings (when the laser doesn’t see anything in range).

3. **z_rand** *(unitless)*  
   - **Default**: 0.5  
   - Weight for random noise in readings.

4. **z_short** *(unitless)*  
   - **Default**: 0.005  
   - Weight for unexpected short readings. For robots encountering lots of small obstacles, increasing can help.

5. **lambda_short** *(unitless)*  
   - **Default**: 0.1  
   - Exponential decay factor for short readings. Higher means the system reacts more strongly to short, unexpected hits.

6. **sigma_hit** *(meters)*  
   - **Default**: 0.2  
   - Standard deviation for the Gaussian part of the sensor model. Higher = more tolerance for variation.

---

### **Motion Model Type**
- **robot_model_type** *(string)*  
  - **Default**: `"nav2_amcl::DifferentialMotionModel"`  
  - For a standard two-wheeled differential drive.  
  - If using an omnidirectional robot, set to `"nav2_amcl::OmniMotionModel"`.

---

### **Update Triggers**
- **update_min_d** *(meters)*  
  - **Default**: 0.25  
  - How far the robot must move before the filter updates. Decreasing to something like 0.05 makes updates more frequent.

- **update_min_a** *(radians)*  
  - **Default**: 0.2  
  - How much the robot must rotate before the filter updates. Lower means more frequent updates.

---

### **Transform Settings**
- **tf_broadcast** *(bool)*  
  - **Default**: True  
  - Tells AMCL to broadcast the `map -> odom` transform. Usually kept `true`.

- **transform_tolerance** *(seconds)*  
  - **Default**: 1.0  
  - How long the transform is valid. A higher value tolerates communication delays but could be less accurate.

---

### **Miscellaneous**
- **scan_topic** *(string)*  
  - **Default**: `"scan"`  
  - Must match your robot’s LIDAR topic name.

- **save_pose_rate** *(Hz)*  
  - **Default**: 0.5  
  - How often (in Hz) to save pose estimates to the parameter server. Higher = more frequent saves.

---

## **Key Takeaways**

1. **Start With Defaults**  
   Most default values work well to start. Adjust them only if your robot’s hardware or environment requires it.

2. **Tune Odometry Noise**  
   Set your `alpha*` parameters so AMCL accurately reflects how reliable your wheel encoders or odometry data are.

3. **Use Enough Particles**  
   If AMCL struggles to locate or maintain the robot’s position, raise `min_particles` or `max_particles` slightly.

4. **Balance Update Frequency**  
   If your robot moves slowly or in tight spaces, decreasing `update_min_d` or `update_min_a` can help maintain precise localization.

5. **Check Laser Model**  
   Make sure `laser_model_type` and associated parameters are suited to your environment (e.g., dynamic vs. static, cluttered vs. open).

6. **Confirm TF Frames**  
   Ensure `base_frame_id`, `odom_frame_id`, and `global_frame_id` match your robot’s TF structure.

---

### **Practical Workflow**
1. **Initial Launch**: Use default parameters and verify basic localization.  
2. **Observe Performance**: Look for drifting, slow updates, or inaccurate pose estimates.  
3. **Adjust Step-by-Step**:  
   - If rotation is off, modify `alpha1` or `alpha2`.  
   - If forward motion is off, adjust `alpha3` or `alpha4`.  
   - If the robot gets “lost” easily, increase `min_particles` and maybe enable recovery (set `recovery_alpha_fast` or `recovery_alpha_slow` above 0).  
4. **Verify**: Test in different parts of the environment to ensure robust performance.

---

# Behavior Tree Navigator (bt_navigator)

`bt_navigator` is the “decision-maker” of the Nav2 stack. It uses **behavior trees** to guide the robot toward a goal or through a series of goals. Think of it like giving step-by-step instructions with conditions:

> *“If the path is blocked, try a recovery. If recovery fails, pick an alternate route. Continue until you reach your goal.”*

By structuring navigation logic in a behavior tree, the robot can dynamically handle unexpected obstacles and other real-world challenges.

### **Main Use Cases**

1. **NavigateToPose**  
   - Moves the robot from its current position to a single destination.  

2. **NavigateThroughPoses**  
   - Guides the robot through multiple intermediate waypoints on the way to a final destination.

Throughout the navigation process, `bt_navigator`:
- Requests and monitors path planning
- Oversees path following
- Triggers recovery behaviors if something goes wrong
- Publishes status updates (e.g., “goal reached,” “obstacle encountered,” “recovery failed,” etc.)

---

## **Key Parameters and Their Meanings**

### 1. **Frames and Topics**

- **global_frame** *(unitless)*  
  - **Default**: `"map"`  
  - **Usage**: This is the reference frame in which the robot’s goals and map data are defined.  
  - **Notes**: Typically remains `"map"` unless you have a unique multi-map or differently named global frame.

- **robot_base_frame** *(unitless)*  
  - **Default**: `"base_link"`  
  - **Usage**: The frame attached to the robot’s body (where all robot-centric transforms originate).  
  - **Notes**: Change only if your robot’s URDF uses a different name. Mismatched frame names cause navigation errors.

- **odom_topic** *(unitless)*  
  - **Default**: `"odom"`  
  - **Your Value**: `"/odometry/filtered"`  
  - **Usage**: The topic that `bt_navigator` uses to receive odometry information.  
  - **Notes**: Using a filtered odometry source (e.g., `robot_localization`) can improve localization accuracy.

---

### 2. **Behavior Tree Timing and Execution**

- **bt_loop_duration** *(milliseconds)*  
  - **Default**: `10`  
  - **Usage**: Determines how often (in ms) the behavior tree “ticks” (i.e., makes decisions).  
  - **Notes**: 
    - Lower values (e.g., 5ms) = faster reaction but higher CPU usage.  
    - Higher values (e.g., 20ms) = slower reaction but more CPU-friendly.

- **default_server_timeout** *(milliseconds)*  
  - **Default**: `20`  
  - **Usage**: How long each BT Action Node waits for its corresponding Action Server to respond before considering it unavailable.  
  - **Notes**: Increase if servers take longer to respond. Decrease to fail fast in a well-tuned system.

- **wait_for_service_timeout** *(milliseconds)*  
  - **Default**: `1000` (1 second)  
  - **Usage**: How long each BT node waits for a service to confirm it’s ready during initialization.  
  - **Notes**: A 1-second wait is usually fine. Raise if your system or network is slow to start services.

- **action_server_result_timeout** *(seconds)*  
  - **Default**: `900.0` (15 minutes)  
  - **Usage**: How long an action server waits before discarding a goal if no result is produced.  
  - **Notes**: Useful if your robot might need a lot of time to complete certain tasks. Lower it to end failed tasks sooner.

---

### 3. **Navigator Plugins**

- **navigators** *(vector\<string\>)*  
  - **Default**: `["navigate_to_pose", "navigate_through_poses"]`  
  - **Usage**: Lists the available behavior tree navigation actions your system supports.  
  - **Notes**:  
    - `"navigate_to_pose"`: Single-goal navigation.  
    - `"navigate_through_poses"`: Multi-waypoint navigation.  
    - You can add more custom navigation types as plugins.

- **error_code_names** *(vector\<string\>)*  
  - **Default**: `["compute_path_error_code", "follow_path_error_code"]`  
  - **Usage**: Tracks specific error codes during navigation.  
  - **Notes**:  
    - Default focuses on path-planning and path-following errors.  
    - Adding `"smoother_error_code"` or others can help diagnose different types of failures.

---

### 4. **Other Important Settings**

- **transform_tolerance** *(seconds)*  
  - **Default**: `0.1`  
  - **Usage**: How old a transform (TF data) can be before `bt_navigator` considers it invalid.  
  - **Notes**:  
    - Increasing helps if you have network or processing delays.  
    - Decreasing requires more up-to-date transforms but can cause issues if messages arrive late.

---

## **Putting It All Together**

1. **Coordinate Frames**  
   - Ensure `global_frame` = `"map"` and `robot_base_frame` = `"base_link"` match your robot’s TF tree.  
   - Confirm your chosen odometry topic (filtered or raw) is correct.

2. **Behavior Tree Execution**  
   - Adjust `bt_loop_duration` to balance responsiveness vs. CPU load.  
   - Tune `default_server_timeout` and `wait_for_service_timeout` to match how quickly or slowly your system components respond.

3. **Navigation Types and Error Tracking**  
   - Use `navigators` to enable or disable certain navigation modes (single-goal vs. multi-waypoint).  
   - Track only the error codes you need to monitor, keeping logs clear and meaningful.

4. **Long Missions & Transform Updates**  
   - If your robot takes a long path or does tasks over many minutes, verify that `action_server_result_timeout` is large enough.  
   - Keep `transform_tolerance` at a level that accommodates network or hardware delays but still ensures accurate localization.

---

# controller_server Overview

The `controller_server` is responsible for converting a planned path into real velocity commands for the robot. It typically:

1. Receives a path from the planner (global navigation).
2. Uses local “controllers” or “plugins” (e.g., DWB, MPPI, Pure Pursuit) to generate velocity commands.
3. Monitors the robot’s progress toward the goal, checking for obstacles, stuck conditions, or success.

---

## **1. General controller_server Parameters**

These parameters apply to the controller node overall, regardless of which plugin you use.

### **controller_frequency** (Hz)
- **Default**: 20.0  
- **Example Custom Value**: 5.0  
- **Meaning**: How frequently (in Hz) the controller runs to compute new velocity commands.  
- **Usage Notes**:  
  - Higher values (e.g., 20 Hz) increase responsiveness but require more CPU.  
  - Lower values (e.g., 5 Hz) reduce CPU load but make the robot react more slowly.

### **costmap_update_timeout** (seconds)
- **Default**: 0.30  
- **Meaning**: Maximum time to wait for the local costmap to update before the controller flags an error.  
- **Usage Notes**:  
  - Increase if your system or sensor updates are slow, but the robot may use slightly outdated obstacle data.  
  - Decrease to force more frequent updates (more responsive to obstacles) but risk timeouts on slower systems.

### **min_x_velocity_threshold** (m/s)
- **Default**: 0.0001  
- **Example Custom Value**: 0.001  
- **Meaning**: Ignores forward/backward velocities below this threshold to filter out small noise.  
- **Usage Notes**:  
  - For a typical indoor robot, 0.001 m/s is enough to ignore tiny movements while still registering normal motion.

### **min_y_velocity_threshold** (m/s)
- **Default**: 0.0001  
- **Example Custom Value**: 0.001  
- **Meaning**: Ignores lateral velocities below this threshold (e.g., mecanum or omnidirectional robots).  
- **Usage Notes**:  
  - Increase for less sensitivity to small sideways motions.  
  - If you’re using a differential drive (no lateral movement), you might keep this very small or default.

### **min_theta_velocity_threshold** (rad/s)
- **Default**: 0.0001  
- **Example Custom Value**: 0.001  
- **Meaning**: Ignores angular velocities below this threshold to filter small rotational noise.  
- **Usage Notes**:  
  - Higher values ignore slow pivots; lower values capture even gentle turning.

### **failure_tolerance** (seconds)
- **Default**: 0.0  
- **Example Custom Value**: 0.3  
- **Meaning**: How long the local controller can fail before the `FollowPath` action itself is considered failed.  
- **Usage Notes**:  
  - `0.0` = fail immediately if a single attempt fails.  
  - `-1.0` = never give up.  
  - Any positive number gives the system time to recover from transient issues.

---

## **2. Progress Checker Configuration**

These parameters help detect if the robot is stuck and not making forward progress.

- **progress_checker_plugin** (string)  
  - **Default**: `"progress_checker"`  
  - **Meaning**: The plugin that monitors robot progress (commonly `SimpleProgressChecker`).

- **progress_checker.required_movement_radius** (m)  
  - **Default**: 0.5  
  - **Meaning**: How far the robot must move before it’s considered “making progress.”  
  - **Usage**: Larger = stricter about movement (risk more false stuck detections). Smaller = more lenient.

- **progress_checker.movement_time_allowance** (s)  
  - **Default**: 10.0  
  - **Meaning**: Time allowed to move `required_movement_radius` before marking the robot as stuck.  
  - **Usage**: Increase for slow or heavy robots. Decrease for a more responsive stuck detection.

---

## **3. Goal Checker Configuration**

### **goal_checker_plugins** (vector\<string\>)
- **Default**: `[ "goal_checker" ]`  
- **Example Custom Value**: `[ "general_goal_checker" ]`  
  - **Meaning**: Which plugins define the “goal reached” conditions.  

#### **general_goal_checker.xy_goal_tolerance** (m)
- **Default**: 0.25  
- **Example**: 0.35  
- **Meaning**: How close in x-y space the robot must be to the goal before we consider it “reached.”  
- **Usage**: Larger = easier to declare success. Smaller = more precise.

#### **general_goal_checker.yaw_goal_tolerance** (rad)
- **Default**: 0.25  
- **Example**: 0.50 (~28.6°)  
- **Meaning**: Angular tolerance. If the robot is within this orientation difference, it’s considered goal-complete.  
- **Usage**: Larger = less spinning to align exactly. Smaller = more precise orientation but can cause “goal dancing.”

---

## **4. Local Controller Plugin Selection**

### **controller_plugins** (vector\<string\>)
- **Default**: `[ "FollowPath" ]`  
- **Meaning**: Which controller plugins are active. Usually just `[ "FollowPath" ]`.  
- **Key Sub-Parameters**:  
  - `FollowPath.plugin`: The actual controller you use under the “FollowPath” name.

#### **Common Plugin Options**
1. **DWB (Dynamic Window Approach)**
   - `"dwb_core::DWBLocalPlanner"`
   - Default local planner for Nav2, often used with differential-drive robots.
2. **MPPI (Model Predictive Path Integral)**
   - `"nav2_mppi_controller::MPPIController"`
   - Samples many possible trajectories, picks the best. Often smoother but more CPU-intensive.
3. **Rotation Shim + Primary Controller (e.g., Regulated Pure Pursuit)**
   - `"nav2_rotation_shim_controller::RotationShimController"`  
   - Then specify `FollowPath.primary_controller` = `"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"` (or another).
   - RotationShim first aligns the robot to the path, then delegates path-following to the chosen controller.

---

## **5. MPPI Controller Parameters**

If you choose `"nav2_mppi_controller::MPPIController"`, you have many advanced parameters that guide sampling and cost functions:

### **Core Sampling Parameters**
- **time_steps**  
  - **Default**: 56  
  - **Example**: 15 (fewer steps = less computation, shorter lookahead)
- **model_dt** (s)  
  - **Default**: 0.05  
  - **Example**: 0.2 (coarser time increments = fewer calculations)
- **batch_size**  
  - **Default**: 1000  
  - **Example**: 10000 (larger = more possible trajectories, higher CPU usage)

### **Velocity Standard Deviations & Limits**  
- **vx_std, vy_std, wz_std**  
  - **Default**: `0.2, 0.2, 0.2`  
  - **Examples**: `vx_std=0.2, vy_std=0.2, wz_std=0.4` (allows more rotational variety)  
- **vx_max** (m/s)  
  - **Default**: 0.5  
  - **Example**: 0.5  
  - The robot’s top forward speed.  
- **vx_min** (m/s)  
  - **Default**: -0.35  
  - **Example**: 0.0 (disallows backward motion)  
- **vy_max** (m/s)  
  - **Default**: 0.5  
- **wz_max** (rad/s)  
  - **Default**: 1.9  

### **Acceleration Limits**  
- **ax_max, ax_min, ay_max, az_max**  
  - **Defaults**: 3.0, -3.0, 3.0, 3.5 respectively  
  - Meaning: forward/back, lateral, and angular acceleration maxima.

### **Iteration & Cost Settings**
- **iteration_count**  
  - **Default**: 1  
  - Typically left at 1 if you have a large `batch_size`.
- **temperature**  
  - **Default**: 0.3  
  - Higher = more random exploration. Lower = picks lower-cost trajectories more deterministically.
- **gamma**  
  - **Default**: 0.015  
  - Weighs how “picky” the system is about cost. Higher = strongly favor best trajectories; Lower = explore more.

---

### **Cost Critics**  
MPPI uses “critics” to evaluate each potential trajectory. Each critic has parameters like `enabled (bool)`, `cost_power (int)`, `cost_weight (double)`, etc.

1. **ConstraintCritic**  
   - **Purpose**: Ensures robot commands are physically feasible (speed/turn limits).
   - **Defaults**: `cost_weight=4.0`, `enabled=true`.
2. **CostCritic**  
   - **Purpose**: Avoids obstacles using the costmap, respecting the robot’s footprint if `consider_footprint=true`.
   - **Defaults**: `cost_weight=3.81`, `critical_cost=300.0`.
   - **Custom**: `consider_footprint=true` improves safety but is more CPU-intensive.
3. **GoalCritic**  
   - **Purpose**: Pulls the robot toward the goal location.  
   - **Default**: `cost_weight=5.0`.
4. **GoalAngleCritic**  
   - **Purpose**: Aligns the robot with the final goal orientation.
   - **Default**: `cost_weight=3.0`.
5. **PathAlignCritic**  
   - **Purpose**: Keeps the robot aligned with the path.  
   - **Default**: `cost_weight=10.0`  
   - **Custom**: `cost_weight=14.0` for stricter path alignment.
6. **PathAngleCritic**  
   - **Purpose**: Maintains correct heading along the path.  
   - **Default**: `cost_weight=2.2`  
   - **Custom**: `cost_weight=2.0` = slightly looser orientation following.
7. **PathFollowCritic**  
   - **Purpose**: Encourages forward progress.  
   - **Default**: `cost_weight=5.0`.
8. **PreferForwardCritic**  
   - **Purpose**: Discourages reverse movement.  
   - **Default**: `cost_weight=5.0`.
9. **TwirlingCritic**  
   - **Purpose**: Prevents excessive spinning in place.  
   - **Default**: `cost_weight=10.0`.

---

## **6. Regulated Pure Pursuit Controller Parameters**

If you choose `"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"` (possibly with `RotationShimController`):

### **Motion & Lookahead**
- **desired_linear_vel** (m/s)  
  - **Default**: 0.5  
  - **Example**: 0.4 for smoother, slower motion.
- **lookahead_dist** (m)  
  - **Default**: 0.6  
  - **Example**: 0.7 for slightly longer anticipation.
- **min_lookahead_dist** (m)  
  - **Default**: 0.3  
  - **Example**: 0.5 for guaranteed minimum path preview.
- **max_lookahead_dist** (m)  
  - **Default**: 0.9  
  - **Example**: 0.7 to avoid looking too far ahead.
- **lookahead_time** (s)  
  - **Default**: 1.5  
  - Determines how far in time the controller projects the robot’s path.

### **Heading & Rotation**
- **rotate_to_heading_angular_vel** (rad/s)  
  - **Default**: 1.8  
  - **Example**: 0.375 for slow, careful turns.
- **use_rotate_to_heading** (bool)  
  - **Default**: true  
  - Whether the robot should pivot in place to face the path direction.
- **rotate_to_heading_min_angle** (rad)  
  - **Default**: 0.785 (~45°)  
  - If the robot’s heading differs by more than this angle, it rotates in place.

### **Velocity Scaling**
- **use_velocity_scaled_lookahead_dist** (bool)  
  - **Default**: false  
  - **Example**: true for adjusting lookahead distance based on velocity.  
- **use_cost_regulated_linear_velocity_scaling** (bool)  
  - **Default**: false  
  - **Example**: true to slow down near obstacles for safety.
- **regulated_linear_scaling_min_radius** (m)  
  - **Default**: 0.9  
  - **Example**: 0.85 = reduce speed if obstacles are within 0.85 m.
- **regulated_linear_scaling_min_speed** (m/s)  
  - **Default**: 0.25  
  - Robot never goes below this speed (unless forced to stop).

### **Approach and Collision Handling**
- **min_approach_linear_velocity** (m/s)  
  - **Default**: 0.05  
  - Slow final approach velocity.
- **approach_velocity_scaling_dist** (m)  
  - **Default**: 1.0  
  - **Example**: 0.6 to begin slowing earlier near the goal.
- **use_collision_detection** (bool)  
  - **Default**: true  
  - Enables collision checks.
- **max_allowed_time_to_collision_up_to_carrot** (s)  
  - **Default**: 1.0  
  - **Example**: 1.5 for more conservative collision avoidance.

---

# **Putting It All Together**

1. **Pick a Controller**  
   - **MPPI** for smooth, flexible control (but higher CPU usage).  
   - **Regulated Pure Pursuit** for simpler, stable path following with good turning.  
   - **DWB** if you want the standard local planner and have a differential-drive robot.

2. **Set Overall Frequency**  
   - Adjust `controller_frequency` to balance responsiveness vs. computational cost.

3. **Tune Progress Checking & Goal Checking**  
   - Keep `progress_checker` values reasonable to detect actual stalls without false alarms.  
   - Choose correct `xy_goal_tolerance` and `yaw_goal_tolerance` for your environment’s precision needs.

4. **Configure Plugin-Specific Parameters**  
   - **MPPI**: Sample sizes, standard deviations, and cost critics.  
   - **Regulated Pure Pursuit**: Lookahead distances, velocity scaling, rotation speeds.

5. **Verify in a Test Environment**  
   - Start with conservative values (slower speeds, bigger goal tolerances).  
   - Gradually refine for faster speeds or tighter accuracy.

6. **Observe and Iterate**  
   - Watch for oscillations near goals, collision near corners, or timeouts if your system is slow.  
   - Make small parameter adjustments to find the perfect balance for your robot and environment.

---

# Local Costmap Overview

The **local_costmap** provides a short-range, high-detail view of the robot’s surroundings. Think of it as a continuously moving “safety bubble” that updates with sensor data (e.g., LIDAR, 3D cameras, ultrasonic sensors). While the **global_costmap** handles the bigger picture for path planning, the local costmap focuses on **immediate** obstacle avoidance and real-time maneuvering.

---

## **1. Core Local Costmap Parameters**

These parameters define the local costmap’s size, resolution, and behavior.

### **update_frequency** (Hz)
- **Default**: 5.0  
- **Purpose**: How often the local costmap processes new sensor data and updates obstacle information.  
- **Trade-off**: Higher frequency = more responsive obstacle updates but heavier on CPU.

### **publish_frequency** (Hz)
- **Default**: 5.0  
- **Purpose**: Rate at which the costmap is published for visualization/debugging.  
- **Trade-off**: Higher = more frequent visualization updates, possibly more CPU usage.

### **global_frame** (string)
- **Default**: `"map"`  
- **Custom Example**: `"odom"`  
- **Purpose**: Sets the reference frame for the local costmap.  
- **Notes**:  
  - Many robots use `"odom"` for the local costmap so it “moves” with the robot.  
  - Using `"map"` can cause additional transforms for local navigation.

### **robot_base_frame** (string)
- **Default**: `"base_link"`  
- **Purpose**: Name of the robot’s main frame (the center of the robot).  
- **Notes**: Ensure it matches your TF setup.

### **rolling_window** (bool)
- **Default**: false  
- **Custom Example**: true  
- **Purpose**: If true, the costmap window moves with the robot instead of staying in a fixed location.  
- **Notes**: Essential for local navigation to keep the area around the robot updated.

### **width, height** (meters)
- **Default**: 5.0 each  
- **Purpose**: Dimensions of the local costmap “window.”  
- **Notes**: 5m x 5m is common for indoor robots. Increase if needed for faster-moving robots.

### **resolution** (m/cell)
- **Default**: 0.1  
- **Custom Example**: 0.05  
- **Purpose**: Grid cell size. Smaller = more detailed obstacle representation but higher CPU cost.

### **robot_radius** (meters)
- **Default**: 0.1  
- **Custom Example**: 0.15  
- **Purpose**: Used for circle-based robots. Helps the costmap account for the robot’s actual size.  
- **Note**: If you have a polygon footprint, you’ll configure it differently. For circle footprints, set the correct radius.

---

## **2. Plugin Layers Overview**

The local costmap typically uses multiple “layers” to blend various sensor inputs and handle obstacles. The **plugins** parameter lists these layers in a specific order. Common layers:

1. **obstacle_layer**  
2. **voxel_layer** (for 3D data)  
3. **range_sensor_layer** (for ultrasonic/IR)  
4. **denoise_layer** (for filtering sensor noise)  
5. **inflation_layer**

Below are the main parameters for each layer.

---

### **2.1 Obstacle Layer**

- **Purpose**: Processes 2D laser scans or similar data to mark obstacles and clear free space in the costmap.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
   - Toggles the obstacle layer on/off.

2. **observation_sources** (vector\<string\>)  
   - **Custom Example**: `scan`  
   - Names of sensor inputs for obstacle detection.

3. **scan.*** Sub-Parameters  
   - `scan.topic`: e.g., `/scan`  
   - `scan.raytrace_min_range`: e.g., 0.20 m  
   - `scan.obstacle_min_range`: e.g., 0.20 m  
   - `scan.max_obstacle_height`: e.g., 2.0 m  
   - `scan.clearing`: true or false (whether to clear free space)  
   - `scan.marking`: true or false (whether to mark obstacles)  
   - `scan.data_type`: `"LaserScan"`  

**Tips**  
- **min_range** filters out very close data that might be inside the robot’s footprint or too noisy.  
- **max_obstacle_height** ensures you only consider obstacles up to a certain height (useful indoors).  
- Enable **clearing** to remove stale obstacle data if the sensor sees free space in that spot.

---

### **2.2 Voxel Layer** (3D Obstacle Layer)

- **Purpose**: Uses 3D data (e.g., from a depth camera) to mark obstacles in a voxel grid, which then projects into the 2D costmap.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
2. **z_voxels** (int)  
   - **Default**: 10  
   - **Custom**: 16 for higher vertical resolution.  
3. **max_obstacle_height** (m)  
   - **Default**: 2.0  
   - Obstacles above this are ignored.  
4. **observation_sources**: e.g., `realsense1`  
   - Sub-params:  
     - `realsense1.topic`: e.g., `/cam_1/depth/color/points`  
     - `realsense1.data_type`: `"PointCloud2"`  
     - `realsense1.obstacle_max_range`: e.g., 1.25 m  
     - `realsense1.marking`: true  
     - `realsense1.clearing`: false (if you don’t do raytracing with 3D data)

**Tips**  
- A higher `z_voxels` or finer `z_resolution` improves vertical resolution but increases computation.  
- Make sure your 3D camera topics match the parameter config (e.g., `PointCloud2`).

---

### **2.3 Range Sensor Layer**

- **Purpose**: Uses ultrasonic or infrared sensors for close-range obstacle detection.  
- **Note**: If you already have laser/3D sensors, range layers can be optional or supplemental.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
   - You can disable if you’re not using ultrasonic sensors.  
2. **topics** (vector\<string\>)  
   - e.g., `["/ultrasonic1"]`  
3. **phi** (double)  
   - **Default**: 1.2 (radians for sensor coverage angle)  
4. **clear_on_max_reading** (bool)  
   - **Default**: false (set true to clear obstacles if sensor reports its max range—helpful for removing false positives).

---

### **2.4 Denoise Layer**

- **Purpose**: Filters out tiny or isolated “ghost” obstacles that appear from sensor noise.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
   - Turn on/off the denoise layer.  
2. **minimal_group_size** (int)  
   - **Default**: 2 (if an obstacle cluster is smaller than 2 cells, it’s considered noise).  
3. **group_connectivity_type** (int)  
   - **Default**: 8 (8-connected cells count as a group).  
4. **Performance Consideration**  
   - The bigger your costmap or the more frequent the updates, the more CPU this can consume.

---

### **2.5 Inflation Layer**

- **Purpose**: Expands obstacles by a certain radius to create a safety buffer. Higher cost near obstacles, gradually decreasing outward. Prevents the robot from navigating dangerously close to obstacles.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
2. **inflation_radius** (m)  
   - **Default**: 0.55  
   - **Custom Example**: 1.75 for extra clearance.  
3. **cost_scaling_factor** (unitless)  
   - **Default**: 1.0  
   - **Custom Example**: 2.58 to increase how fast costs inflate near obstacles.  
4. **inflate_unknown** (bool)  
   - **Default**: false  
   - Whether unknown cells are treated as obstacles and inflated. Usually false for indoor.

**Tips**  
- A large **inflation_radius** helps keep the robot away from walls and furniture but may reduce navigable space.  
- A higher **cost_scaling_factor** means obstacles have a “steeper” cost gradient, discouraging paths close to them.

---

## **Recommended Plugin Order**

A common plugin sequence for the local costmap might look like this (in `plugins:` parameter):

1. **obstacle_layer** (for 2D laser data)  
2. **voxel_layer** (for 3D depth data)  
3. **range_sensor_layer** (for ultrasonics, if used)  
4. **denoise_layer** (for filtering out noisy readings)  
5. **inflation_layer** (final pass to expand obstacles)

Remember: **Order matters.** Typically, you want to mark obstacles, remove noise, then inflate.

---

# **Bringing It All Together**

1. **Size & Frequency**  
   - Set `width` and `height` to cover the area around the robot.  
   - Use `rolling_window=true` so the costmap stays centered on the robot.  
   - Use `resolution=0.05` for fine obstacle detail or `0.1` for reduced CPU load.

2. **Reference Frames**  
   - `global_frame="odom"` is common for local navigation so that the costmap moves smoothly with the robot.  
   - `robot_base_frame="base_link"` typically remains unchanged unless your TF tree differs.

3. **Layer Configuration**  
   - **Obstacle Layer**: Enable marking and possibly clearing if you use a laser scanner.  
   - **Voxel Layer**: Enable if you have 3D data (like RealSense). Tune `z_voxels` and `obstacle_max_range` carefully.  
   - **Range Sensor Layer**: Enable if you have ultrasonic/IR sensors; disable otherwise.  
   - **Denoise Layer**: Helps remove “speckle” noise from LIDAR or camera data.  
   - **Inflation Layer**: Adjust radius and cost scale to set how strictly the robot avoids obstacles.

4. **Test & Iterate**  
   - Visualize your local costmap (via RViz) to see if obstacles show up as expected.  
   - Check for “ghost” obstacles or missing objects.  
   - Adjust plugin settings (e.g., `min_obstacle_height`, `max_obstacle_height`, `inflation_radius`) to refine performance.

---

# Global Costmap Overview

The **global_costmap** is a long-range map of the environment that the robot uses for path planning:

- **Static Map**: Permanent features (e.g., walls, doorways) from a SLAM or provided map.  
- **Sensor Updates**: Obstacles detected by LIDAR, depth cameras, or other sensors.  
- **Robot Pose**: Provided by localization (e.g., AMCL) in the `map` frame.

The output is a 2D grid where cells can be:
- **Free**: Robot can travel here.
- **Occupied**: Contains obstacles or high-cost areas.
- **Unknown**: Unmapped areas (if `track_unknown_space` is enabled).

A well-tuned global costmap ensures the robot can plan end-to-end paths avoiding all known obstacles.

---

## **1. Core Global Costmap Parameters**

These settings define the fundamental behavior and resolution of your global costmap.

### **update_frequency** (Hz)
- **Default**: 5.0  
- **Purpose**: How often sensor data is incorporated into the costmap.  
- **Notes**: 5 Hz is typical and usually sufficient for a static or semi-static environment.

### **publish_frequency** (Hz)
- **Default**: 1.0  
- **Example Custom Value**: 5.0  
- **Purpose**: How often the global costmap is published for visualization or debugging.  
- **Notes**: Increasing from 1 Hz to 5 Hz allows more frequent updates in RViz, but uses more network/CPU.

### **global_frame** (string)
- **Default**: `"map"`  
- **Purpose**: The coordinate frame in which the global costmap resides (usually the static world frame).

### **robot_base_frame** (string)
- **Default**: `"base_link"`  
- **Purpose**: The robot’s main (base) coordinate frame.

### **robot_radius** (meters)
- **Default**: 0.1  
- **Example**: 0.15  
- **Purpose**: Radius of the robot if using a circle-based footprint. Ensures costmap layers keep an appropriate clearance.

### **resolution** (m/cell)
- **Default**: 0.1  
- **Example**: 0.05  
- **Purpose**: Size of each grid cell. Smaller (0.05) yields a finer map but increases memory and CPU load.

### **track_unknown_space** (bool)
- **Default**: false  
- **Example**: true  
- **Purpose**: If `true`, the costmap distinguishes cells that have never been observed from free/occupied cells.  
- **Notes**: Helps the planner avoid unknown areas unless it must explore them.

---

## **2. Plugins List**

The **plugins** parameter controls which layers are active in the global costmap and in what order they’re applied. A typical set might look like:

```yaml
plugins:
  - static_layer
  - obstacle_layer
  - voxel_layer
  - range_sensor_layer
  - inflation_layer
```

Each layer merges its data (e.g., obstacles or cost inflation) into the final global costmap.

---

## **3. Static Layer**

- **Purpose**: Loads and holds the static map (walls, hallways, etc.)—the “floor plan” for the robot.

**Key Parameters**  
1. **map_subscribe_transient_local** (bool)  
   - **Default**: true  
   - Determines how the map is subscribed to. `true` uses “transient local” durability, receiving new map data published after the node starts.

**Why It’s Important**  
- Ensures the robot has a base understanding of permanent obstacles and navigable areas.

---

## **4. Obstacle Layer**

- **Purpose**: Updates the global costmap with obstacles from 2D sensors like LIDAR.  
- **Typical “observation_source”**: `scan`

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
   - Toggles the obstacle layer on/off.
2. **observation_sources** (string)  
   - e.g., `"scan"`
3. **scan.topic** (string)  
   - e.g., `"/scan"`
4. **scan.obstacle_min_range** / **scan.raytrace_min_range**  
   - e.g., `0.20` m (filters out very close readings, since the robot body is near the LIDAR)
5. **scan.max_obstacle_height**  
   - e.g., `2.0` m
6. **scan.clearing** (bool)  
   - **Default**: false  
   - If `true`, frees up space where the sensor sees no obstacles.
7. **scan.marking** (bool)  
   - **Default**: true  
   - Marks new obstacles as occupied.
8. **scan.data_type**: `"LaserScan"`  

**Tips**  
- Use clearing + marking to maintain a more accurate global map (especially if objects can appear/disappear).

---

## **5. Voxel Layer**

- **Purpose**: Uses 3D data (e.g., from an RGB-D camera or stereo camera) to mark elevated obstacles.  
- **Applies a 3D grid (voxel map)** and projects it into the 2D costmap.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
2. **publish_voxel_map** (bool)  
   - **Default**: false (publish or not the internal 3D map for debugging)
3. **z_voxels** (int), **z_resolution** (float)  
   - Control vertical resolution. More voxels = finer detail = higher CPU.
4. **observation_sources**: e.g., `robot_depth_camera`  
   - sub-params like `topic: /rgbd_camera`, `raytrace_max_range: 1.25`, etc.
5. **clearing** / **marking**  
   - Typically `marking=true`, `clearing=false` if you only want to add obstacles; set `clearing=true` if you want to remove them based on free-space data.

**Tips**  
- Limit `obstacle_max_range` to avoid unnecessary large-range scanning in global costmap if you want to keep CPU usage lower.  
- The voxel layer can slow down path planning if overused with high resolution + large range.

---

## **6. Range Sensor Layer**

- **Purpose**: Incorporates sensor data from ultrasonics or IR sensors in the global map.  
- **Similar** to the obstacle layer, but specialized for range sensors with a cone-shaped field of view.

**Key Parameters**  
1. **enabled** (bool)  
   - **Default**: true  
   - If `false`, this layer will not modify the costmap.
2. **topics** (vector of strings)  
   - e.g., `["/ultrasonic1"]`
3. **phi** (double)  
   - Angular width of the sensor’s coverage.
4. **clear_on_max_reading** (bool)  
   - If `true`, remove obstacles when sensor reports max range (free space).
5. **mark_threshold**, **clear_threshold**  
   - Probability thresholds used to mark/clear cells.

**Tips**  
- If you don’t actually have ultrasonic sensors, you might disable this layer.  
- Using it in the global costmap may be less common if the sensor only covers a small range (useful if obstacles can appear anywhere unexpectedly).

---

## **7. Inflation Layer**

- **Purpose**: Expands the obstacles outward to create a safety buffer.  
- **Key** to avoiding collisions by steering the robot away from walls and obstacles.

**Key Parameters**  
1. **inflation_radius** (m)  
   - **Default**: 0.55  
   - **Example**: 1.75  
   - Determines how far from an obstacle the “cost” is spread.
2. **cost_scaling_factor** (double)  
   - **Default**: 1.0  
   - **Example**: 2.58  
   - Controls how quickly cost drops off with distance from an obstacle.

**Tips**  
- A larger inflation radius gives more clearance but can make narrow passages impassable.  
- A higher cost_scaling_factor means obstacles quickly become “very costly” to approach.

---

# **Putting It All Together**

1. **Global Map Resolution**  
   - Decide if you need 0.05 m/cell or if 0.1 m/cell is enough. Higher resolution = more memory/CPU usage.

2. **Choose and Order Layers**  
   - Typically:  
     1. **static_layer** (base map)  
     2. **obstacle_layer** (LIDAR)  
     3. **voxel_layer** (3D camera)  
     4. **range_sensor_layer** (ultrasonic)  
     5. **inflation_layer** (final safety expansion)  

3. **Consider CPU Load**  
   - The global costmap is typically updated less frequently than the local costmap.  
   - Using multiple sensor layers (voxel + obstacle + range) at high resolution can be resource-intensive.

4. **Align Robot Frame & Footprint**  
   - Confirm `robot_base_frame` matches your TF tree.  
   - Ensure `robot_radius` (or footprint polygon) is correct for your robot’s real dimensions.

5. **Enable/Disable Dynamic Updates**  
   - If your environment is fairly static, you may not need real-time clearing/marking on the global costmap.  
   - If things change often, enable marking+clearing so the global map stays accurate over time.

---

# Map Saver Overview

The **map_saver** package allows you to store the robot’s current view of the environment into permanent map files. It does this by converting the in-memory occupancy grid (which the robot builds or updates as it explores) into two files:

1. **.pgm File**  
   - A grayscale image representation of the map:  
     - **White** = free space  
     - **Black** = occupied space (walls/obstacles)  
     - **Gray** = unknown space

2. **.yaml File**  
   - Contains metadata about the map, such as:  
     - **resolution**: The size of each cell (meters/cell).  
     - **origin**: The map’s reference point in the world.  
     - **occupied_thresh** and **free_thresh**: Probability thresholds for deciding which cells become black or white in the .pgm file.  

This saved map can be used later for localization and navigation (e.g., by AMCL and Nav2), much like saving a floor plan for future reference.

---

## **1. Core Parameters**

### **save_map_timeout** (seconds)  
- **Default**: 2.0  
- **Example Custom Value**: 5.0  
- **Meaning**: The maximum time the node will wait to complete the map saving process before timing out.  
- **Usage Notes**:  
  - **Longer Timeout (e.g., 5.0s)**: Good for large or detailed maps, ensuring you don’t lose partial data.  
  - **Shorter Timeout (e.g., 2.0s)**: Faster saving but might fail with big maps or slower hardware.

---

### **free_thresh_default** (probability)  
- **Default**: 0.25  
- **Meaning**: The probability below which a cell is considered “free” (white) in the .pgm file.  
- **Usage Notes**:  
  - If the robot’s internal map says there’s a **25% or less chance** of an obstacle, that cell is saved as free.  
  - **Higher Value (e.g., 0.3)**: Stricter on free space, so the map will mark fewer cells as free (some borderline cells become unknown or occupied).  
  - **Lower Value (e.g., 0.2)**: More lenient, labeling more cells as free.

---

### **occupied_thresh_default** (probability)  
- **Default**: 0.65  
- **Meaning**: The probability above which a cell is considered “occupied” (black) in the .pgm file.  
- **Usage Notes**:  
  - If the robot’s internal map says there’s a **65% or greater chance** an obstacle is present, that cell is marked as occupied.  
  - **Higher Value (e.g., 0.7)**: More conservative about marking cells as obstacles, so you might see fewer black cells (only the ones the robot is very sure about).  
  - **Lower Value (e.g., 0.6)**: More aggressive about considering cells occupied.

---

## **2. How It Works**

1. **Map Data Collection**  
   - As the robot explores, it builds an occupancy grid.  
2. **map_saver Trigger**  
   - You call `map_saver` to capture the current state of this occupancy grid.  
3. **Saving**  
   - The node writes a `.pgm` (image) and `.yaml` (metadata) file.  
   - The thresholds (`free_thresh_default`, `occupied_thresh_default`) decide how cells are colored in the `.pgm` file.  
   - The `save_map_timeout` gives the system a window to complete the operation.

---

## **3. Practical Tips**

- **Check Robot Size**: A large environment or high map resolution can mean bigger files. If saves time out, raise `save_map_timeout`.  
- **Tune Thresholds Carefully**:  
  - If your `free_thresh_default` is too high, some genuinely free cells might appear unknown or occupied.  
  - If your `occupied_thresh_default` is too low, you may get extra black pixels where the robot is not fully certain an obstacle exists.  
- **Use Consistent Resolution**: Ensure the `.yaml` resolution matches the map’s resolution in your navigation stack.  

---

# Planner Server

**Purpose**  
The planner_server calculates an initial path from the robot’s current location to the goal using the **global_costmap**. It ensures the path is collision-free (based on known obstacles), tries to minimize path length, and respects the robot’s motion constraints.

## **Key Parameters**

1. **expected_planner_frequency** (Hz)  
   - **Default**: 20.0  
   - **Meaning**: How frequently (in Hz) the planner is expected to run.  
   - **Notes**: 20 Hz is common; lowering it may reduce CPU load but slows responsiveness.

2. **planner_plugins** (list of strings)  
   - **Default**: `[ "GridBased" ]`  
   - **Meaning**: Lists which planner algorithms to load.  
   - **Notes**: Each plugin has a configuration section. Common planners:
     - **NavfnPlanner** (“GridBased”) – A classic Dijkstra-based approach.  
     - **SmacPlanner** – More advanced (A*, hybrids).  
     - **ThetaStarPlanner** – Tries to shorten paths around corners.

3. **GridBased.plugin** (string)  
   - **Default**: `"nav2_navfn_planner::NavfnPlanner"`  
   - **Meaning**: Specifies the actual plugin class to use for path planning.  

4. **GridBased.tolerance** (meters)  
   - **Default**: 0.5  
   - **Meaning**: How close the robot must get to the goal before planning is considered successful.  
   - **Notes**: Larger tolerance = easier to achieve “goal reached,” but less precise stopping.

5. **GridBased.use_astar** (bool)  
   - **Default**: false  
   - **Meaning**: If `true`, A* is used instead of Dijkstra’s algorithm for pathfinding.  
   - **Notes**: A* can be faster in certain grids, but Dijkstra’s (default) is typically adequate for most scenarios.

6. **GridBased.allow_unknown** (bool)  
   - **Default**: true  
   - **Meaning**: If `true`, the planner can path through unknown areas in the map.  
   - **Notes**: Setting to `false` restricts paths to only known free space, preventing exploration into unmapped regions.

---

# **Smoother Server**

**Purpose**  
The smoother_server refines the “raw” path from the planner to create more natural, flowing routes. It can round off sharp turns and optimize segments so the robot moves efficiently and smoothly.

## **Key Parameters**

1. **smoother_plugins** (vector<string>)  
   - **Default**: `[ "simple_smoother" ]`  
   - **Meaning**: Which smoothing algorithms to load.  
   - **Notes**: Multiple plugins can be listed if you want different smoothing options. Typically, one is enough.

2. **simple_smoother.plugin** (string)  
   - **Default**: `"nav2_smoother::SimpleSmoother"`  
   - **Meaning**: The specific smoother class to use.  
   - **Notes**: This plugin simply and reliably smooths corners and reduces path jitter.

3. **simple_smoother.tolerance** (double)  
   - **Default**: `1.0e-10`  
   - **Meaning**: The precision threshold before the smoother stops refining.  
   - **Notes**: A very small tolerance yields extremely smooth paths but may take more computation.

4. **simple_smoother.max_its** (integer)  
   - **Default**: Not specified (in some setups)  
   - **Example Value**: `1000`  
   - **Meaning**: The maximum number of iterations to attempt while smoothing.  
   - **Notes**: Higher = more thorough smoothing, but more CPU time.

---

## **Putting It All Together**

1. **Pick a Planner Plugin**  
   - Stick with `NavfnPlanner` if you need a traditional, stable approach.  
   - Use `SmacPlanner` or other advanced methods if you need more robust or feature-rich pathfinding (e.g., hybrid-A* for car-like robots).

2. **Refine the Path**  
   - **Smoother Server** helps avoid jerkiness. If the planner creates a path with sharp turns, smoothing will make the motion more fluid.

3. **Parameter Tuning**  
   - **Tolerance** parameters: Larger values make planning/smoothing faster but potentially less precise.  
   - **Frequency** (e.g., 20 Hz vs. 10 Hz): Higher demands more CPU but quickly updates paths.  
   - **Iterations** in smoothing: Too high can impact performance if you have a large environment, but yields smoother paths.

4. **Test & Observe**  
   - Visualize in RViz. Watch for paths with unnecessary tight corners or excessive detours.  
   - Adjust tolerance, use_astar, or the smoothing iteration counts for best results.

With a well-tuned **planner_server** and **smoother_server**, your robot will generate optimized routes that it can follow comfortably and efficiently, respecting both global constraints (obstacles, goal location) and local motion considerations (path curvature, smoothness).

---

# Behavior Server Overview

The **Behavior Server** provides specialized maneuvers (“behaviors”) the robot can perform during navigation. While the **BT Navigator** coordinates overall navigation (e.g., “navigate to room B”), the Behavior Server handles specific, focused actions (e.g., “spin in place” or “back up”). Each behavior is a plugin with its own logic for movement, safety checks, and execution flow.

**Typical Use Cases**  
- **Spin**: Rotate in place to reorient or look around.  
- **Backup**: Safely move backward if stuck.  
- **Drive on Heading**: Move straight along a given heading for a certain distance.  
- **Wait**: Pause autonomously for a specified time.  
- **Assisted Teleop**: Combine user control with collision avoidance.

---

## **1. Costmap and Footprint Topics**

These topics ensure each behavior has up-to-date information about obstacles and the robot’s footprint.

1. **local_costmap_topic** (string)  
   - **Default**: `"local_costmap/costmap_raw"`  
   - **Meaning**: Where the behavior server gets the local costmap data for immediate surroundings.

2. **global_costmap_topic** (string)  
   - **Default**: `"global_costmap/costmap_raw"`  
   - **Meaning**: The global costmap data source, giving a broader map of the environment.

3. **local_footprint_topic** (string)  
   - **Default**: `"local_costmap/published_footprint"`  
   - **Meaning**: Provides the robot’s local footprint for collision checks.

4. **global_footprint_topic** (string)  
   - **Default**: `"global_costmap/published_footprint"`  
   - **Meaning**: Provides the robot’s footprint in global coordinates.

---

## **2. Frequency and Behaviors**

1. **cycle_frequency** (Hz)  
   - **Default**: 10.0  
   - **Meaning**: Rate at which the Behavior Server updates (checking costmaps, sending commands, etc.).  
   - **Notes**: 10 Hz is typical for most robots to ensure smooth updates without overloading CPU.

2. **behavior_plugins** (vector\<string\>)  
   - **Default**: `[ "spin", "backup", "drive_on_heading", "wait" ]`  
   - **Example**: `[ "spin", "backup", "drive_on_heading", "assisted_teleop", "wait" ]`  
   - **Meaning**: Which specialized behaviors to load at runtime.  
   - **Notes**: Add or remove plugins based on your robot’s needs.

---

## **3. Kinematic & Safety Parameters**

These parameters control motion limits during behaviors (especially spin, backup, and drive-on-heading).

1. **simulate_ahead_time** (seconds)  
   - **Default**: 2.0  
   - **Meaning**: How far into the future the behavior server checks for collisions with the costmap.  
   - **Notes**: 2.0s is a standard lookahead time.

2. **max_rotational_vel** (rad/s)  
   - **Default**: 1.0  
   - **Example**: 0.5  
   - **Meaning**: The maximum allowed angular velocity when spinning or turning.  
   - **Notes**: Lower = slower spins but safer in tight environments.

3. **min_rotational_vel** (rad/s)  
   - **Default**: 0.4  
   - **Meaning**: The minimum angular velocity to keep the robot rotating.  
   - **Notes**: If too low, the robot might stall; if too high, it might rotate too quickly.

4. **rotational_acc_lim** (rad/s^2)  
   - **Default**: 3.2  
   - **Meaning**: The max angular acceleration limit, controlling how quickly the robot can spin up or slow down.

---

## **4. Command Type & Frames**

1. **enable_stamped_cmd_vel** (bool)  
   - **Default**: true in newer versions; false in older versions  
   - **Meaning**: If `true`, uses `TwistStamped` messages (with timestamps and frames). If `false`, uses basic `Twist`.  
   - **Notes**: TwistStamped can be beneficial for timing-sensitive or frame-specific control. Basic Twist is simpler.

2. **local_frame** (string)  
   - **Default**: `"odom"`  
   - **Meaning**: The frame to use for local movements (e.g., tracking the robot’s position in local space).

3. **global_frame** (string)  
   - **Default**: `"map"`  
   - **Meaning**: The frame used for global references (like absolute coordinates in the known environment).

4. **robot_base_frame** (string)  
   - **Default**: `"base_link"`  
   - **Meaning**: The reference frame for the robot itself, typically at the center or main body of the robot.

5. **transform_timeout** (seconds)  
   - **Default**: 0.1  
   - **Meaning**: How long the Behavior Server waits for a valid transform lookup before timing out.  
   - **Notes**: 0.1s is enough for most real-time systems; increase if transforms occasionally arrive late.

---

# How the Behavior Server Fits in Nav2

1. **BT Navigator**: Coordinates higher-level logic.  
2. **Behavior Server**: Executes specific actions (spin, backup, etc.) when requested by the behavior tree.  
3. **Costmaps**: Provide real-time obstacle data to ensure safe maneuvers.

This separation allows the Behavior Server to be efficient and focused on its “specialist” tasks without complicating the main navigation tree.

---

# Waypoint Follower Overview

The **waypoint_follower** package enables the robot to visit a series of waypoints in sequence. It handles:

- **Navigating** to each point in order.  
- **Reporting success or failure** at each waypoint.  
- **Optionally performing tasks** (e.g., waiting, taking a picture) at each waypoint via a plugin interface.

This is particularly useful for patrols, inspection routines, or any application requiring multiple checkpoints.

---

## **1. Core Parameters**

1. **loop_rate** (Hz)
   - **Default**: 20  
   - **Example**: 2 (custom)  
   - **Purpose**: How often (in Hz) the waypoint follower checks progress toward each waypoint.  
   - **Notes**:  
     - A higher rate (like 20 Hz) makes progress checks frequent but uses more CPU.  
     - A lower rate (e.g., 2 Hz) usually suffices, reducing load while still being responsive.

2. **stop_on_failure** (bool)
   - **Default**: true  
   - **Example**: false  
   - **Purpose**: Determines whether the robot stops the entire waypoint mission if it fails to reach one waypoint.  
   - **Notes**:  
     - `true`: Mission stops upon a single failure.  
     - `false`: Robot continues to the next waypoint even if one is missed (useful in dynamic or uncertain environments).

3. **global_frame_id** (string)
   - **Default**: “map”  
   - **Meaning**: The reference frame used for waypoint coordinates.  
   - **Notes**: Typically “map” is standard; no change often needed.

4. **action_server_result_timeout** (seconds)
   - **Default**: 900.0 (15 minutes)  
   - **Purpose**: How long to wait for a navigation action result (e.g., reaching a waypoint) before discarding the goal.  
   - **Notes**: 900 seconds allows long tasks to complete without timing out prematurely.

---

## **2. Waypoint Task Executor Plugin**

The waypoint follower can run **custom code** at each waypoint—like pausing, capturing sensor data, or controlling a manipulator. By default, it uses a “wait_at_waypoint” plugin that simply pauses the robot.

### **waypoint_task_executor_plugin** (string)
- **Default**: `wait_at_waypoint`
- **Meaning**: The name of the plugin to run at each waypoint.

#### **wait_at_waypoint.plugin** (string)
- **Default**: `"nav2_waypoint_follower::WaitAtWaypoint"`
- **Meaning**: Specifies the actual plugin class for “wait_at_waypoint.”

#### **wait_at_waypoint.enabled** (bool)
- **Default**: Not specified (assumed true if not mentioned)
- **Example**: true  
- **Meaning**: Whether to activate the waiting behavior at waypoints.

#### **wait_at_waypoint.waypoint_pause_duration** (seconds)
- **Default**: Not specified  
- **Example**: 10  
- **Purpose**: How long the robot pauses at each waypoint.  
- **Notes**: If your robot needs time to “settle,” gather sensor data, or interact with the environment, configure this accordingly.

---

## **Putting It All Together**

Below is a simplified YAML snippet illustrating how you might set these parameters:

```yaml
waypoint_follower:
  loop_rate: 2
  stop_on_failure: false
  action_server_result_timeout: 900.0
  waypoint_task_executor_plugin: "wait_at_waypoint"

  wait_at_waypoint:
    plugin: "nav2_waypoint_follower::WaitAtWaypoint"
    enabled: true
    waypoint_pause_duration: 10
```

1. **Loop rate** = 2 Hz: Checks progress less often, reducing CPU use.  
2. **Stop on failure** = false: Continues waypoint mission even if one is missed.  
3. **Wait at waypoint** = 10s: Robot waits 10 seconds at each waypoint.

---

# Velocity Smoother Overview

The **velocity_smoother** node ensures smooth, gradual acceleration and deceleration of the robot. Without it, raw velocity commands (such as those from a joystick or navigation controller) might cause sudden starts and stops. Such abrupt movements:

1. **Increase Wear** on drive motors and hardware.  
2. **Make the Robot Less Stable**, especially for tall or high-speed robots.  
3. **Reduce Control Precision** because large, sudden velocity changes can introduce mechanical oscillations.

By controlling how quickly the robot speeds up or slows down, velocity_smoother improves safety and hardware longevity.

---

## **1. Frequency and Feedback Mode**

1. **smoothing_frequency** (Hz)  
   - **Default**: 20.0  
   - **Meaning**: How often the velocity_smoother updates velocity commands to gradually reach the target velocity.  
   - **Notes**: 20 Hz is common. Higher frequencies provide finer control but use more CPU.

2. **feedback** (string)  
   - **Default**: `"OPEN_LOOP"`  
   - **Meaning**:  
     - **OPEN_LOOP**: Smooth using fixed acceleration/deceleration.  
     - **CLOSED_LOOP**: Uses odometry feedback for more accurate speed tracking.  
   - **Notes**: If you set `feedback: "CLOSED_LOOP"`, make sure `odom_topic` is correct so the smoother can read actual velocities.

3. **scale_velocities** (bool)  
   - **Default**: false  
   - **Meaning**: Whether to scale velocities proportionally under certain conditions. Often not needed unless special constraints require it.

---

## **2. Velocity Limits**

1. **max_velocity** (m/s or rad/s)  
   - **Default**: `[0.5, 0.0, 2.5]` for (x, y, theta)  
   - **Example Custom**: `[0.5, 0.5, 2.5]`  
   - **Meaning**: Maximum linear speed (x, y) and maximum angular speed (theta).  
   - **Notes**:  
     - If your robot is **holonomic**, set the y-value as well.  
     - If your robot is **differential drive**, keep the y-component at 0.0 because it cannot move sideways.

2. **min_velocity** (m/s or rad/s)  
   - **Default**: `[-0.5, 0.0, -2.5]` (x, y, theta)  
   - **Example Custom**: `[-0.5, -0.5, -2.5]`  
   - **Meaning**: Minimum (negative) speeds allowed for reversing or rotating in the opposite direction.  
   - **Notes**: Holonomic robots might want a negative y-velocity; differential drive typically sets y to 0.

3. **deadband_velocity** (m/s or rad/s)  
   - **Default**: `[0.0, 0.0, 0.0]`  
   - **Meaning**: If velocities are within this range, they’re considered zero.  
   - **Notes**: Often left at `[0,0,0]` unless your hardware requires a minimum threshold to overcome friction.

4. **velocity_timeout** (seconds)  
   - **Default**: 1.0  
   - **Meaning**: If no new commands arrive within this time, the smoother stops publishing velocities for safety.

---

## **3. Acceleration and Deceleration**

1. **max_accel** (m/s² or rad/s²)  
   - **Default**: `[2.5, 0.0, 3.2]`  
   - **Example Custom**: `[0.3, 0.3, 3.2]`  
   - **Meaning**: Positive acceleration limits for x, y, and theta.  
   - **Notes**: Lower (e.g., 0.3) means gentler acceleration, easier on motors.

2. **max_decel** (m/s² or rad/s²)  
   - **Default**: `[-2.5, 0.0, -3.2]`  
   - **Example Custom**: `[-0.5, -0.5, -3.2]`  
   - **Meaning**: Negative values for deceleration.  
   - **Notes**: Lower deceleration magnitude (e.g., -0.5) = gentler stops, preventing abrupt halts.

---

## **4. Other Configuration Details**

1. **odom_topic** (string)  
   - **Default**: `"odom"`  
   - **Example Custom**: `"odometry/filtered"`  
   - **Meaning**: The odometry topic the smoother listens to (relevant for CLOSED_LOOP feedback).  
   - **Notes**: If you’re using robot_localization for fused odometry, set it here.

2. **odom_duration** (seconds)  
   - **Default**: 0.1  
   - **Meaning**: Window of odometry data used to compute current velocity in CLOSED_LOOP mode.

3. **use_realtime_priority** (bool)  
   - **Default**: false  
   - **Meaning**: If `true`, runs the smoother with real-time scheduling priority—generally not needed unless you have strict timing requirements.

4. **enable_stamped_cmd_vel** (bool)  
   - **Default**: `true` (in newer versions) or `false` (in older versions)  
   - **Meaning**: If `true`, the smoother publishes `TwistStamped` with timestamps and frames. Otherwise, it uses a basic `Twist`.

---

# **Putting It All Together**

Here’s an example YAML snippet:

```yaml
velocity_smoother:
  smoothing_frequency: 20.0
  feedback: "OPEN_LOOP"
  scale_velocities: false

  max_velocity: [0.5, 0.5, 2.5]
  min_velocity: [-0.5, -0.5, -2.5]
  deadband_velocity: [0.0, 0.0, 0.0]
  velocity_timeout: 1.0

  max_accel: [0.3, 0.3, 3.2]
  max_decel: [-0.5, -0.5, -3.2]

  odom_topic: "odometry/filtered"
  odom_duration: 0.1
  use_realtime_priority: false
  enable_stamped_cmd_vel: false
```

1. **Holonomic Robot**: Notice non-zero y-velocity limits.  
2. **Gentle Accelerations**: `max_accel` at 0.3 helps minimize jerk.  
3. **Slow Decel**: `max_decel` at -0.5 for safe, gradual stops.

---

# Collision Monitor Overview

The **collision_monitor** node is an extra safety layer in the Nav2 stack. It:

1. **Monitors Sensor Data** (e.g., laser scans) in real time, bypassing the usual costmap or planner.  
2. **Identifies Potential Collisions** ahead of time using “safety zones,” which can be polygons, circles, or dynamic shapes.  
3. **Intervenes** by modifying or halting velocity commands if an obstacle is detected in a critical zone, preventing accidents.

This is especially valuable in environments where robots move quickly or work near humans, as it reacts more rapidly than a full navigation cycle.

---

## **1. Frame and Time Parameters**

1. **base_frame_id** (string)  
   - **Default**: `"base_footprint"`  
   - **Meaning**: The main reference frame of the robot’s body. Used as the center for collision checking.  
   - **Notes**: Change only if your robot’s URDF uses a different name (e.g., `"base_link"`).

2. **odom_frame_id** (string)  
   - **Default**: `"odom"`  
   - **Meaning**: The odometry frame used for robot movement tracking.  
   - **Notes**: Typically unchanged unless your TF tree uses a different name.

3. **transform_tolerance** (seconds)  
   - **Default**: 0.1  
   - **Example**: 0.2  
   - **Meaning**: How old transform data (TF) can be before it’s considered stale.  
   - **Notes**: Higher = more tolerant of TF delays, but might use outdated transforms.

4. **source_timeout** (seconds)  
   - **Default**: 2.0  
   - **Example**: 1.0  
   - **Meaning**: How long collision_monitor waits for sensor data before declaring it stale.  
   - **Notes**: Lower = stricter about fresh sensor data (react faster but can drop data quickly); higher = more tolerant of slower sensors.

---

## **2. Velocity Command Topics**

1. **cmd_vel_in_topic** (string)  
   - **Default**: `"cmd_vel_smoothed"`  
   - **Meaning**: The input velocity command topic to check for collisions.  
   - **Notes**: Typically the velocity from a smoother or navigation controller.

2. **cmd_vel_out_topic** (string)  
   - **Default**: `"cmd_vel"`  
   - **Meaning**: The topic on which collision_monitor publishes “safety-approved” velocity commands.  
   - **Notes**: Usually the robot’s motor driver or controller listens to this final `cmd_vel`.

3. **state_topic** (string)  
   - **Default**: `""` (empty)  
   - **Example**: `"collision_monitor_state"`  
   - **Meaning**: If set, publishes debugging/status info about which safety zone triggers are active.

---

## **3. Polygon and Zone Definition**

Collision Monitor uses “polygons” (or circles) to define safety zones. For example, you can have:

- **Stop Zone**: If an obstacle enters, the robot halts.  
- **Slowdown Zone**: The robot slows if obstacles are there.  
- **Approach Zone**: Checks “time-to-collision” and caps speed accordingly.

### **Example Polygon: FootprintApproach**

1. **polygons** (vector\<string\>)  
   - **Meaning**: List of polygon definitions. Each entry is a different zone or safety behavior.  

2. **FootprintApproach.type** (string)  
   - **Example**: `"polygon"`  
   - **Meaning**: Use a polygon shaped like the robot’s footprint.  
   - **Other Options**: `"circle"` or dynamic shapes based on velocity.

3. **FootprintApproach.time_before_collision** (seconds)  
   - **Default**: 2.0  
   - **Example**: 1.2  
   - **Meaning**: How far into the future (time-to-collision) the monitor checks for obstacles.  
   - **Notes**: Larger = more conservative stops/slowdowns.

4. **FootprintApproach.simulation_time_step** (seconds)  
   - **Default**: 0.1  
   - **Meaning**: How often the collision prediction runs during the check window.  
   - **Notes**: Smaller step = more accurate but higher CPU usage.

---

## **4. Sensor Sources**

1. **observation_sources** (vector\<string\>)  
   - **Meaning**: Which sensors feed obstacle data into collision_monitor. E.g., `[ "scan" ]`.  
   - **Notes**: Can be laser, pointcloud, or range sensor sources—each with its own sub-parameters.

2. **base_shift_correction** (bool)  
   - **Default**: true  
   - **Meaning**: Whether to account for robot motion when using sensor data.  
   - **Notes**: Keep `true` for moving robots (especially if they’re fast). `false` might be okay for slow or stationary robots, saving CPU.

---

## **5. Example Setup**

Here’s a sample YAML snippet that demonstrates the key parameters:

```yaml
collision_monitor:
  base_frame_id: "base_footprint"
  odom_frame_id: "odom"
  transform_tolerance: 0.2
  source_timeout: 1.0

  cmd_vel_in_topic: "cmd_vel_smoothed"
  cmd_vel_out_topic: "cmd_vel"
  state_topic: "collision_monitor_state"

  base_shift_correction: true

  polygons: ["FootprintApproach"]

  FootprintApproach:
    type: "polygon"
    time_before_collision: 1.2
    simulation_time_step: 0.1

  observation_sources: ["scan"]
  scan:
    topic: "/scan"
    max_obstacle_height: 2.0
    ...
    # (Typical laser scan config here)
```

**How it works**:

- **Collision Monitor** listens for velocity commands on `cmd_vel_smoothed`.  
- It checks if those commands would lead to collisions within **1.2 seconds** ahead.  
- If collision is likely, it modifies or zeroes out the velocity before publishing to `cmd_vel`.  
- Only LIDAR data on `/scan` is used for detecting obstacles in this setup.

---

# Docking Server Overview

The **docking_server** orchestrates the robot’s approach, alignment, and final connection (or disconnection) to a docking station. It:

1. **Approaches** a pre-staging area near the dock.
2. **Aligns** the robot using sensors (e.g., fiducial markers, IR, ultrasonic, etc.).
3. **Executes Final Approach** at a carefully controlled speed for precise alignment.
4. **Monitors** sensors or signals to confirm successful docking.
5. **Undocks** by reversing the process, ensuring a safe departure.

This plugin-based system allows customization for:

- Different **robot types** (differential drive, omnidirectional, etc.).
- Various **docking mechanisms** (magnetic, contact plates, etc.).
- Multiple **sensor inputs** (vision-based markers, range sensors, fiducials).

---

## **1. Typical Parameter Categories**

Below are the main categories of parameters you might see in a **docking_server** YAML file. If you don’t have a dock set up, you can keep default or placeholder values until you’re ready to implement real-world docking.

### **1.1 Docking Workflow**

1. **dock_pre_staging_distance** (meters)  
   - **Meaning**: How far from the docking station the robot will stop initially before doing a precise alignment.  
   - **Notes**: Larger distances (e.g., 0.5–1.0 m) can help if your alignment method needs space to detect the dock or fiducial.

2. **dock_approach_speed** (m/s)  
   - **Meaning**: Speed for the robot’s final approach. Slower speeds = more control, less chance of overshoot.  
   - **Notes**: Typically 0.05–0.1 m/s for precise docking.

3. **dock_alignment_tolerance** (meters / radians)  
   - **Meaning**: Positional/orientational error allowed during final approach.  
   - **Notes**: For charging plates, you might need very tight tolerances (~0.01–0.02 m). For looser magnetic docks, you can allow more.

4. **dock_timeout** (seconds)  
   - **Meaning**: Maximum time to attempt docking before failing.  
   - **Notes**: If alignment or approach takes too long, the system aborts and logs an error.

### **1.2 Undocking Workflow**

1. **undock_reverse_distance** (meters)  
   - **Meaning**: How far to back away from the dock once disconnected.  
   - **Notes**: Enough distance to ensure the robot is clear of the docking mechanism.

2. **undock_speed** (m/s)  
   - **Meaning**: Speed for backing up out of the dock.  
   - **Notes**: Often the same or slightly faster than docking speed, e.g., 0.1–0.2 m/s.

3. **undock_timeout** (seconds)  
   - **Meaning**: Time limit for completing undocking.  
   - **Notes**: Prevents the robot from getting stuck in an error state.

### **1.3 Sensor / Plugin Configuration**

1. **docking_plugins** (list of strings)  
   - **Meaning**: Which plugin(s) provide the logic for detecting and aligning to the dock.  
   - **Examples**:  
     - `"fiducial_docking_plugin"` for camera-based fiducial detection.  
     - `"ir_docking_plugin"` for IR-based alignment.

2. **sensor_topics** (dictionary)  
   - **Meaning**: Input sensors used for detecting the dock location or alignment markers.  
   - **Examples**: A camera topic, an IR range topic, etc.

3. **alignment_controller**  
   - **Meaning**: Specifies how the robot adjusts orientation and position to match the dock approach angle.  
   - **Notes**: Could be a simple “rotate in place” approach or a more advanced local planner for smooth alignment.

### **1.4 General Behavior / Debugging**

1. **simulation_mode** (bool)  
   - **Meaning**: If `true`, certain commands or sensors might be mocked for testing.  
   - **Notes**: Useful for verifying logic without physically docking.

2. **verbose** (bool)  
   - **Meaning**: Toggles extra logging or debug prints.  
   - **Notes**: Helpful while tuning alignment or speed parameters.

3. **action_server_result_timeout** (seconds)  
   - **Default**: ~900.0 (15 minutes) in many Nav2 servers  
   - **Meaning**: How long the docking server will hold the goal action handle if the docking sequence doesn’t complete in time.

---

## **2. Example Minimal YAML**

Below is a simplified example demonstrating how some parameters might look for a robot using a vision-based alignment plugin:

```yaml
docking_server:
  ros__parameters:
    # Workflow Parameters
    dock_pre_staging_distance: 0.5
    dock_approach_speed: 0.05
    dock_alignment_tolerance: 0.02  # meters
    dock_timeout: 120  # 2 minutes

    undock_reverse_distance: 0.3
    undock_speed: 0.1
    undock_timeout: 60  # 1 minute

    # Plugin and Sensor Configuration
    docking_plugins: ["fiducial_docking_plugin"]

    fiducial_docking_plugin:
      plugin: "my_robot_docking::FiducialDockPlugin"
      camera_topic: "/camera/image_raw"
      fiducial_topic: "/fiducial_transforms"

    # Behavior / Debug
    simulation_mode: false
    verbose: true

    # Action Timeout
    action_server_result_timeout: 900.0
```

1. **dock_pre_staging_distance** = 0.5 m: Robot stops half a meter from the station to scan for alignment.  
2. **dock_approach_speed** = 0.05 m/s: Very slow, controlled final approach.  
3. **undock_reverse_distance** = 0.3 m: Backs up 0.3 m after undocking.  
4. **docking_plugins**: Just “fiducial_docking_plugin,” referencing a custom or existing plugin that uses camera data to align with a fiducial marker.

---

## **3. General Workflow**

1. **Approach & Pre-Staging**:  
   - The docking server navigates the robot to a known “pre-dock” pose ~0.5–1.0 m away from the station.

2. **Sensor-Based Alignment**:  
   - The selected docking plugin reads sensor data (vision, IR, etc.) to adjust heading and lateral position.

3. **Final Approach**:  
   - Moves in slowly within **dock_alignment_tolerance**.  
   - The server monitors any feedback from the dock or sensors to confirm a secure connection.

4. **Docking Completion**:  
   - Robot is declared “docked.” Possibly lowers speeds, stops motors, or triggers charging.

5. **Undocking** (reverse sequence):  
   - The robot carefully reverses out of the dock area at a controlled speed.



