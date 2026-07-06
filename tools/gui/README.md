# Cell GUI options

The controller publishes machine-readable state (no GUI required to consume it):

| Topic | Type | Content |
|---|---|---|
| `/mating/phase` | `std_msgs/String` | Current phase (latched — late joiners get it) |
| `/mating/error_mm` | `std_msgs/Float64` | Live position error, mm |
| `/mating/error_deg` | `std_msgs/Float64` | Live rotation error, deg |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health: phase, vision freshness, plan failures, insertion enable |

Reset service: `/connector_mating_node/reset` (`std_srvs/Trigger`).
Runtime toggle: `enable_insertion` bool parameter on `/connector_mating_node`
(read fresh each cycle, so flipping it takes effect immediately).

## Option A — Foxglove Studio (recommended for engineering use)

```bash
sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Open Foxglove Studio → *Open connection* → `ws://<robot-pc>:8765`, then
*Layout → Import from file* → `foxglove_layout.json`. You get the camera
view, error plot, phase indicator, diagnostics, and a reset button.
Layout schemas vary slightly between Foxglove versions — if a panel comes
up empty, click its settings and reselect the topic, then re-export the
layout to this file.

## Option B — Operator panel (kiosk/demo use, zero installs)

```bash
python3 tools/gui/mating_panel.py                     # default config
python3 tools/gui/mating_panel.py --config my.yaml    # customized
```

stdlib tkinter + rclpy only. Everything is configured in
[panel_config.yaml](panel_config.yaml): topics, tolerance lines, phase
colours, plot window, and the button row. Buttons are data — add as many as
you like:

```yaml
buttons:
  - label: RESET sequence          # call any std_srvs/Trigger service
    type: service_trigger
    service: /connector_mating_node/reset
  - label: Toggle insertion enable # flip any boolean parameter
    type: param_toggle
    node: /connector_mating_node
    param: enable_insertion
```

## Option C — rqt (nothing to configure)

```bash
rqt   # add: Image View (/aruco/debug_image), Plot (/mating/error_mm/data),
      # Topic Monitor (/mating/phase), Service Caller (reset)
```

**Safety note:** all of these are conveniences. The e-stop is hardware; no
GUI stop is a safety function.
