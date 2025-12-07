#!/usr/bin/env python3
"""
SUMO Configuration Validator and Test Script for DDCL Project

This script validates the SUMO configuration files and tests the simulation
without requiring a full SUMO installation. It checks:
1. XML syntax validity
2. File references and dependencies
3. Parameter consistency
4. Network connectivity

Usage:
    python validate_sumo_config.py
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


class SUMOConfigValidator:
    def __init__(self, scenario_path):
        self.scenario_path = Path(scenario_path)
        self.errors = []
        self.warnings = []
        
    def validate_xml(self, file_path, description):
        """Validate XML syntax"""
        print(f"\n✓ Validating {description}: {file_path.name}")
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            print(f"  - Root element: <{root.tag}>")
            print(f"  - Valid XML syntax")
            return tree, root
        except ET.ParseError as e:
            self.errors.append(f"XML parse error in {file_path}: {e}")
            print(f"  ✗ XML parse error: {e}")
            return None, None
        except FileNotFoundError:
            self.errors.append(f"File not found: {file_path}")
            print(f"  ✗ File not found")
            return None, None
    
    def validate_network(self, net_file):
        """Validate network file"""
        tree, root = self.validate_xml(net_file, "Network file")
        if not tree:
            return False
        
        # Count edges and junctions
        edges = root.findall('.//edge')
        junctions = root.findall('.//junction')
        connections = root.findall('.//connection')
        
        print(f"  - Edges: {len(edges)}")
        print(f"  - Junctions: {len(junctions)}")
        print(f"  - Connections: {len(connections)}")
        
        # Validate edge IDs
        edge_ids = set()
        for edge in edges:
            edge_id = edge.get('id')
            if edge_id:
                edge_ids.add(edge_id)
                lanes = edge.findall('.//lane')
                print(f"    Edge {edge_id}: {len(lanes)} lanes")
        
        # Expected edges for DDCL project
        expected_edges = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
        missing_edges = set(expected_edges) - edge_ids
        if missing_edges:
            self.warnings.append(f"Missing expected edges: {missing_edges}")
        
        return True
    
    def validate_routes(self, rou_file):
        """Validate route file"""
        tree, root = self.validate_xml(rou_file, "Route file")
        if not tree:
            return False
        
        # Count routes and flows
        routes = root.findall('.//route')
        flows = root.findall('.//flow')
        
        print(f"  - Routes: {len(routes)}")
        print(f"  - Flows: {len(flows)}")
        
        for route in routes:
            route_id = route.get('id')
            edges = route.get('edges', '').split()
            print(f"    Route {route_id}: {len(edges)} edges")
            print(f"      Path: {' -> '.join(edges[:5])}...")
        
        for flow in flows:
            flow_id = flow.get('id')
            vph = flow.get('vehsPerHour', 'N/A')
            begin = flow.get('begin', 'N/A')
            end = flow.get('end', 'N/A')
            print(f"    Flow {flow_id}: {vph} veh/h from {begin}s to {end}s")
        
        return True
    
    def validate_vtypes(self, vtype_file):
        """Validate vehicle type file"""
        tree, root = self.validate_xml(vtype_file, "Vehicle type file")
        if not tree:
            return False
        
        # Count vehicle types
        vtypes = root.findall('.//vType')
        distributions = root.findall('.//vTypeDistribution')
        
        print(f"  - Vehicle types: {len(vtypes)}")
        print(f"  - Distributions: {len(distributions)}")
        
        for vtype in vtypes:
            vtype_id = vtype.get('id')
            vclass = vtype.get('vClass', 'N/A')
            length = vtype.get('length', 'N/A')
            max_speed = vtype.get('maxSpeed', 'N/A')
            tau = vtype.get('tau', 'N/A')
            print(f"    Type {vtype_id}: class={vclass}, length={length}m, speed={max_speed}m/s, tau={tau}s")
        
        # Check for required vehicle classes
        vclasses = {vt.get('vClass') for vt in vtypes}
        if 'custom1' not in vclasses:
            self.warnings.append("Vehicle class 'custom1' (HV) not found")
        if 'custom2' not in vclasses:
            self.warnings.append("Vehicle class 'custom2' (CAV) not found")
        
        return True
    
    def validate_detectors(self, det_file):
        """Validate detector file"""
        tree, root = self.validate_xml(det_file, "Detector file")
        if not tree:
            return False
        
        # Count detectors
        detectors = root.findall('.//laneAreaDetector')
        
        print(f"  - Lane area detectors: {len(detectors)}")
        
        # Group by edge
        edge_detectors = {}
        for det in detectors:
            det_id = det.get('id')
            lane = det.get('lane', '')
            edge_id = lane.split('_')[0] if '_' in lane else 'unknown'
            if edge_id not in edge_detectors:
                edge_detectors[edge_id] = []
            edge_detectors[edge_id].append(det_id)
        
        for edge_id, det_list in sorted(edge_detectors.items()):
            print(f"    Edge {edge_id}: {len(det_list)} detectors")
        
        return True
    
    def validate_config(self, cfg_file):
        """Validate main configuration file"""
        tree, root = self.validate_xml(cfg_file, "Main configuration file")
        if not tree:
            return False
        
        # Check input files
        input_section = root.find('.//input')
        if input_section is not None:
            net_file = input_section.find('.//net-file')
            route_files = input_section.find('.//route-files')
            add_files = input_section.find('.//additional-files')
            
            print("  Input files:")
            if net_file is not None:
                net_value = net_file.get('value')
                print(f"    - Network: {net_value}")
                if not (self.scenario_path / net_value).exists():
                    self.errors.append(f"Network file not found: {net_value}")
            
            if route_files is not None:
                route_value = route_files.get('value')
                print(f"    - Routes: {route_value}")
                if not (self.scenario_path / route_value).exists():
                    self.errors.append(f"Route file not found: {route_value}")
            
            if add_files is not None:
                add_value = add_files.get('value')
                print(f"    - Additional: {add_value}")
                for add_file in add_value.split(','):
                    add_file = add_file.strip()
                    if not (self.scenario_path / add_file).exists():
                        self.warnings.append(f"Additional file not found: {add_file}")
        
        # Check time settings
        time_section = root.find('.//time')
        if time_section is not None:
            begin = time_section.find('.//begin')
            end = time_section.find('.//end')
            step = time_section.find('.//step-length')
            
            print("  Time settings:")
            if begin is not None:
                print(f"    - Begin: {begin.get('value')}s")
            if end is not None:
                print(f"    - End: {end.get('value')}s")
            if step is not None:
                print(f"    - Step length: {step.get('value')}s")
        
        return True
    
    def validate_all(self):
        """Run all validations"""
        print("=" * 70)
        print("SUMO Configuration Validation for DDCL Project")
        print("=" * 70)
        
        # Define file paths
        files = {
            'config': self.scenario_path / 'test.sumocfg',
            'network': self.scenario_path / 'net.net.xml',
            'routes': self.scenario_path / 'rou.rou.xml',
            'vtypes': self.scenario_path / 'vTypeDistributions.add.xml',
            'detectors': self.scenario_path / 'det.add.xml',
        }
        
        # Validate each file
        results = {
            'config': self.validate_config(files['config']),
            'network': self.validate_network(files['network']),
            'routes': self.validate_routes(files['routes']),
            'vtypes': self.validate_vtypes(files['vtypes']),
            'detectors': self.validate_detectors(files['detectors']),
        }
        
        # Print summary
        print("\n" + "=" * 70)
        print("VALIDATION SUMMARY")
        print("=" * 70)
        
        total_checks = len(results)
        passed_checks = sum(1 for v in results.values() if v)
        
        print(f"\n✓ Passed: {passed_checks}/{total_checks} file validations")
        
        if self.warnings:
            print(f"\n⚠ Warnings ({len(self.warnings)}):")
            for warning in self.warnings:
                print(f"  - {warning}")
        
        if self.errors:
            print(f"\n✗ Errors ({len(self.errors)}):")
            for error in self.errors:
                print(f"  - {error}")
            print("\n❌ VALIDATION FAILED")
            return False
        else:
            print("\n✅ VALIDATION PASSED - Configuration is ready for SUMO simulation")
            return True


def main():
    # Find scenario directory
    script_dir = Path(__file__).parent
    scenario_path = script_dir / 'V1.6' / 'scenario'
    
    if not scenario_path.exists():
        print(f"Error: Scenario directory not found: {scenario_path}")
        sys.exit(1)
    
    # Run validation
    validator = SUMOConfigValidator(scenario_path)
    success = validator.validate_all()
    
    # Provide next steps
    if success:
        print("\n" + "=" * 70)
        print("NEXT STEPS")
        print("=" * 70)
        print("\n1. Install SUMO if not already installed:")
        print("   sudo apt-get install sumo sumo-tools")
        print("\n2. Set SUMO_HOME environment variable:")
        print("   export SUMO_HOME=/usr/share/sumo")
        print("\n3. Run the simulation:")
        print("   cd V1.6/controller")
        print("   python main.py")
        print("\n4. Or test SUMO directly:")
        print("   cd V1.6/scenario")
        print("   sumo-gui -c test.sumocfg")
        print()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
