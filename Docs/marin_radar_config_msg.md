# Marin_radar_contrl_msgs/msg/RadarControlSet

```yaml
---
items:
    - name: status
        value: transmit
        label: Status
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - standby
            - transmit
    - name: range
        value: '50'
        label: Range
        type: 0
        min_value: 25.0
        max_value: 75000.0
        enums: []
    - name: mode
        value: harbor
        label: Mode
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - custom
            - harbor
            - offshore
            - weather
            - bird
    - name: gain
        value: '35.686275'
        label: Gain
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: sea_clutter
        value: auto
        label: Sea clutter
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: auto_sea_clutter_nudge
        value: '0'
        label: Auto sea clut adj
        type: 0
        min_value: -50.0
        max_value: 50.0
        enums: []
    - name: sea_state
        value: rough
        label: Sea state
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - calm
            - moderate
            - rough
    - name: rain_clutter
        value: '0.784314'
        label: Rain clutter
        type: 0
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: noise_rejection
        value: medium
        label: Noise rejection
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: target_expansion
        value: low
        label: Target expansion
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: interference_rejection
        value: medium
        label: Interf. rej
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: target_separation
        value: low
        label: Target separation
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: scan_speed
        value: 'off'
        label: Fast scan
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - medium
            - high
    - name: doppler_mode
        value: 'off'
        label: VelocityTrack
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - normal
            - approaching_only
    - name: doppler_speed
        value: '2.000000'
        label: Speed threshold
        type: 0
        min_value: 0.05000000074505806
        max_value: 15.949999809265137
        enums: []
    - name: antenna_height
        value: '4.000000'
        label: Antenna height
        type: 0
        min_value: 0.0
        max_value: 30.174999237060547
        enums: []
    - name: bearing_alignment
        value: '0.000000'
        label: Bearing alignment
        type: 0
        min_value: 0.0
        max_value: 360.0
        enums: []
    - name: sidelobe_suppression
        value: auto
        label: Sidelobe sup.
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: lights
        value: 'off'
        label: Halo light
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
---
items:
    - name: status
        value: transmit
        label: Status
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - standby
            - transmit
    - name: range
        value: '50'
        label: Range
        type: 0
        min_value: 25.0
        max_value: 75000.0
        enums: []
    - name: mode
        value: harbor
        label: Mode
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - custom
            - harbor
            - offshore
            - weather
            - bird
    - name: gain
        value: '35.686275'
        label: Gain
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: sea_clutter
        value: auto
        label: Sea clutter
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: auto_sea_clutter_nudge
        value: '0'
        label: Auto sea clut adj
        type: 0
        min_value: -50.0
        max_value: 50.0
        enums: []
    - name: sea_state
        value: rough
        label: Sea state
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - calm
            - moderate
            - rough
    - name: rain_clutter
        value: '0.784314'
        label: Rain clutter
        type: 0
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: noise_rejection
        value: medium
        label: Noise rejection
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: target_expansion
        value: low
        label: Target expansion
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: interference_rejection
        value: medium
        label: Interf. rej
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: target_separation
        value: low
        label: Target separation
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
    - name: scan_speed
        value: 'off'
        label: Fast scan
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - medium
            - high
    - name: doppler_mode
        value: 'off'
        label: VelocityTrack
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - normal
            - approaching_only
    - name: doppler_speed
        value: '2.000000'
        label: Speed threshold
        type: 0
        min_value: 0.05000000074505806
        max_value: 15.949999809265137
        enums: []
    - name: antenna_height
        value: '4.000000'
        label: Antenna height
        type: 0
        min_value: 0.0
        max_value: 30.174999237060547
        enums: []
    - name: bearing_alignment
        value: '0.000000'
        label: Bearing alignment
        type: 0
        min_value: 0.0
        max_value: 360.0
        enums: []
    - name: sidelobe_suppression
        value: auto
        label: Sidelobe sup.
        type: 1
        min_value: 0.0
        max_value: 100.0
        enums: []
    - name: lights
        value: 'off'
        label: Halo light
        type: 2
        min_value: 0.0
        max_value: 0.0
        enums:
            - 'off'
            - low
            - medium
            - high
---
```