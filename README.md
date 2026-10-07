# Energy-Aware Routing Strategy for Mobile UAV Swarms

## Overview

This project implements an Energy-Aware Routing Strategy in ns-3 using C++ for a mobile UAV swarm.

The system dynamically reroutes network traffic away from UAVs with critically low battery levels. It considers both network distance and remaining energy when selecting routes.

The proposed energy-aware routing strategy is evaluated against a conventional distance-based routing baseline.

## Key Features

- 12-node mobile UAV swarm
- Wireless ad-hoc communication
- Heterogeneous initial battery levels
- Energy-aware route selection
- Automatic avoidance of critically low-energy UAVs
- Dynamic route recomputation every 2 seconds
- Dijkstra-based routing
- Wi-Fi energy consumption model
- UDP traffic generation
- FlowMonitor-based performance evaluation
- Energy and routing event logging
- Baseline versus energy-aware comparison

## Simulation Configuration

- Number of UAVs: 12
- Simulation time: 200 seconds
- Nominal initial energy: 500 J
- Critical energy threshold: 75 J (15%)
- Wi-Fi range: 90 m
- Route recomputation interval: 2 seconds
- Wi-Fi standard: 802.11g
- Wi-Fi data rate: 6 Mbps
- UDP flows: 12
- Flow data rate: 128 kbps
- Packet size: 256 bytes
- Energy weight: 4
- Distance weight: 1

## Energy-Aware Routing

The routing algorithm considers both link distance and the remaining battery energy of UAVs.

A UAV whose remaining energy falls below the critical threshold is treated as unavailable for forwarding traffic.

For other UAVs, the routing cost increases as their battery becomes depleted. This encourages traffic to use healthier UAVs and reduces repeated forwarding through low-energy nodes.

The route cost combines:

- Base routing cost
- Link distance
- Remaining battery energy

## Initial Battery Configuration

To test the ability of the routing strategy to protect low-energy UAVs, different initial battery levels are assigned.

- UAV 1: 160 J
- UAV 5: 175 J
- UAV 6: 190 J
- All other UAVs: 500 J

The critical energy threshold is 75 J.

## Baseline

The baseline uses conventional distance-based routing without considering UAV battery energy.

It is used as a reference to determine whether the proposed energy-aware strategy can preserve battery energy and prevent UAVs from becoming critically depleted.



## Results

### Baseline

- Minimum remaining energy: 73.90 J
- UAVs below critical threshold: 1
- Time to first critical UAV: 139.154 s
- PDR: 54.84%
- Throughput: 923.81 kbps
- Average delay: 88.83 ms
- Total remaining energy: 3723.50 J

### Energy-Aware Routing

- Minimum remaining energy: 85.20 J
- UAVs below critical threshold: 0
- PDR: 43.57%
- Throughput: 734.02 kbps
- Average delay: 96.12 ms
- Total remaining energy: 3812.78 J

## Conclusion

The proposed energy-aware routing strategy dynamically changes forwarding decisions based on UAV battery conditions.

Compared with the baseline, the energy-aware approach prevents UAVs from crossing the critical energy threshold during the 200-second simulation and preserves more total residual energy.

The results demonstrate that energy-aware routing can improve battery preservation and support longer UAV swarm operation, while introducing a trade-off in packet delivery ratio, throughput, and delay.
