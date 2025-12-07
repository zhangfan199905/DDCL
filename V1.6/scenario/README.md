# SUMO Scenario Files - DDCL Project

This directory contains all SUMO (Simulation of Urban Mobility) configuration files for the Dynamic CAV Dedicated Lane (DDCL) simulation.

## File Descriptions

### Core Configuration Files

1. **test.sumocfg** - Main SUMO configuration file
   - Entry point for simulation
   - References all other configuration files
   - Sets simulation time, step length, and output options

2. **net.net.xml** - Network definition
   - Highway geometry (4000m total length)
   - 14 edges (segments) with varying number of lanes
   - Junction definitions and connections
   - Lane attributes (width, speed limits, vehicle classes)

3. **rou.rou.xml** - Routes and vehicle flows
   - Vehicle flow definitions
   - Departure rates and timing
   - Route assignments (origin-destination)

4. **vTypeDistributions.add.xml** - Vehicle type parameters
   - HV (Human Vehicle) characteristics
   - CAV (Connected Autonomous Vehicle) characteristics
   - Mixed traffic distribution (67% HV, 33% CAV)

5. **det.add.xml** - Detector configurations
   - Lane area detectors for each segment
   - Data collection frequency (60 seconds)
   - Output file specification

6. **viewsettings.xml** - SUMO-GUI visualization settings
   - Display preferences for simulation visualization
   - Camera position and zoom level
   - Vehicle and lane rendering options

### Output Files (Generated during simulation)

7. **e2_lane_output.xml** - Detector output data
   - Traffic flow measurements
   - Speed, occupancy, and throughput data
   - Time-series performance metrics

8. **fcdTrajectories.xml** - Vehicle trajectory data
   - Individual vehicle positions over time
   - Speed and acceleration profiles
   - Lane change events

9. **lanechanges.xml** - Lane change events
   - All lane change maneuvers
   - Timestamps and locations
   - Vehicle IDs and types

## Network Structure

```
Edge Layout (Total: 4000m):
┌────────────────────────────────────────────────────────────────┐
│  Upstream  │        Control Area (10 segments)        │ Btlnk │ Down │
│   1000m    │              2000m (200m each)            │ 300m  │ 700m │
├────────────┼──────────────────────────────────────────┼───────┼──────┤
│  Edge 1    │ Edges 2-11 (Dynamic Control Zone)        │Edge 12│Edge13│
│  3 lanes   │ 3 lanes (Lane 0 = controllable)          │2 lanes│3 lanes│
└────────────┴──────────────────────────────────────────┴───────┴──────┘

Lane Assignment:
  Lane 2 (Left):   [═════ Regular Mixed Traffic ═════]
  Lane 1 (Middle): [═════ Buffer/Transition ═════════]
  Lane 0 (Right):  [═════ Controllable (CDL/HML/RML) ═]
```

## Vehicle Types

### HV (Human Vehicle) - vClass: custom1
```xml
<vType id="hv" vClass="custom1" 
       length="5.0" width="1.8" minGap="2.5" 
       maxSpeed="33.33" accel="2.6" decel="4.5" 
       tau="1.21" sigma="0.5"/>
```
- **Reaction time (tau)**: 1.21 seconds (human perception-reaction)
- **Driver imperfection (sigma)**: 0.5 (moderate variability)
- **Acceleration**: 2.6 m/s² (typical passenger car)
- **Deceleration**: 4.5 m/s² (comfortable braking)

### CAV (Connected Autonomous Vehicle) - vClass: custom2
```xml
<vType id="cav" vClass="custom2" 
       length="5.0" width="1.8" minGap="2.0" 
       maxSpeed="33.33" accel="3.0" decel="4.5" 
       tau="0.5" sigma="0.0"/>
```
- **Reaction time (tau)**: 0.5 seconds (sensor processing delay)
- **Driver imperfection (sigma)**: 0.0 (perfect control)
- **Acceleration**: 3.0 m/s² (enhanced capability)
- **Deceleration**: 4.5 m/s² (same as HV for safety)

## Traffic Demand

Flow configuration generates mixed traffic:

```xml
<!-- Main flow: 2600 veh/hour for 1800 seconds -->
<flow id="flow_main" type="mixedTraffic" route="highway_route" 
      begin="0" end="1800" vehsPerHour="2600"/>

<!-- Peak flow: Additional 1000 veh/hour during peak period -->
<flow id="flow_peak" type="mixedTraffic" route="highway_route" 
      begin="600" end="1200" vehsPerHour="1000"/>
```

**Expected results:**
- Total vehicles: ~1300-1500 (mix of HV and CAV)
- Peak period: Minutes 10-20 (higher density)
- Vehicle distribution: ~67% HV, ~33% CAV

## Running the Simulation

### Option 1: With Python Controller (Recommended)
```bash
cd ../controller
python main.py
```

### Option 2: SUMO-GUI (Network Testing)
```bash
sumo-gui -c test.sumocfg
```

### Option 3: Command-line SUMO (Batch Processing)
```bash
sumo -c test.sumocfg --no-warnings
```

## Detector Data Collection

Detectors (det.add.xml) measure:
- **sampledSeconds**: Total time vehicles spent in detector
- **nVehEntered/Left**: Vehicle counts
- **meanSpeed**: Average speed (m/s)
- **meanTimeLoss**: Average time lost vs. free-flow
- **meanOccupancy**: Lane occupancy percentage
- **maxJamLength**: Maximum queue length

Data collection frequency: 60 seconds (aligned with GMARL decision cycle)

## Troubleshooting

### "Could not load network" error
- Check that net.net.xml exists in this directory
- Verify XML syntax is valid
- Ensure all edge IDs in routes match network definition

### "Unknown vehicle class" error
- Verify vClass="custom1" and vClass="custom2" are used consistently
- Check that vTypeDistributions.add.xml is loaded in test.sumocfg
- Ensure lane permissions allow custom1 and custom2

### "Route not found" error
- Confirm route "highway_route" is defined in rou.rou.xml
- Check all edges in route exist in network
- Verify edge connections are valid (sequential)

### High collision rates
- Adjust collision.action in test.sumocfg
- Review vehicle headway (minGap) parameters
- Check lane change aggressiveness parameters

## Customization

### Modify Traffic Demand
Edit `rou.rou.xml`:
```xml
<flow id="flow_main" ... vehsPerHour="NEW_VALUE"/>
```

### Change CAV Penetration Rate
Edit `vTypeDistributions.add.xml`:
```xml
<vType id="hv" probability="0.5"/>   <!-- 50% HV -->
<vType id="cav" probability="0.5"/>  <!-- 50% CAV -->
```

### Adjust Network Geometry
Edit `net.net.xml` lane lengths, speeds, or number of lanes.
**Note**: Detector positions in det.add.xml must be updated accordingly.

### Modify Simulation Duration
Edit `test.sumocfg`:
```xml
<time>
    <end value="3000"/>  <!-- Change from 2100 to 3000 seconds -->
</time>
```

## SUMO Version Compatibility

These files are compatible with:
- SUMO 1.16.0 or later
- Tested with SUMO 1.19.0

Check your version:
```bash
sumo --version
```

## Additional Resources

- SUMO Documentation: https://sumo.dlr.de/docs/
- Network Files: https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html
- Vehicle Types: https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html
- Detectors: https://sumo.dlr.de/docs/Simulation/Output/Lanearea_Detectors_(E2).html
- TraCI: https://sumo.dlr.de/docs/TraCI.html
