#!/usr/bin/env python3
"""
Example script demonstrating how to use the GO2 robot with cane sensors.

This script shows how to:
1. Load the GO2 model with a rigidly attached cane
2. Access force-torque sensor data from the cane-robot connection
3. Apply external forces to the cane
4. Simulate the robot compensating for external forces using the force feedback
"""

import numpy as np
import mujoco
import mujoco.viewer
import time

def main():
    # Load the GO2 model with cane
    model = mujoco.MjModel.from_xml_path('go2.xml')
    data = mujoco.MjData(model)
    
    # Get sensor and body IDs
    cane_force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_force')
    cane_torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_torque')
    cane_base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cane_base')
    
    print("Sensor IDs:")
    print(f"  cane_force_id: {cane_force_id}")
    print(f"  cane_torque_id: {cane_torque_id}")
    print(f"  cane_base_id: {cane_base_id}")
    
    # Simulation parameters
    simulation_time = 0.0
    total_time = 5.0  # seconds
    force_magnitude = 10.0  # Newtons
    
    print("\nStarting simulation...")
    print("The robot will experience external forces on the cane handle.")
    print("The cane is rigidly attached to the robot, so forces are transmitted to the base.")
    print("Watch as the robot adjusts its balance to compensate for the external forces.\n")
    
    step_count = 0
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while simulation_time < total_time:
            # Apply time-varying force to cane body
            # Even though cane is rigidly attached, forces are still applied to it
            # and measured by the force-torque sensors at the connection point
            t = simulation_time
            
            # Create a sinusoidal force pattern
            force_x = force_magnitude * np.sin(2 * np.pi * t / total_time)
            force_z = -force_magnitude * 0.5 * np.cos(2 * np.pi * t / total_time)
            
            # Apply force to cane base (which includes the handle geom)
            data.xfrc_applied[cane_base_id, 0] = force_x  # x-direction force
            data.xfrc_applied[cane_base_id, 1] = 0.0      # y-direction force
            data.xfrc_applied[cane_base_id, 2] = force_z  # z-direction force (mostly downward)
            
            # Step simulation
            mujoco.mj_step(model, data)
            simulation_time += model.opt.timestep
            
            # Print sensor data every 100 steps
            if step_count % 100 == 0:
                # Get sensor data
                # The sensor data layout is determined by the sensor definition order
                # For force sensor: 3 components (Fx, Fy, Fz)
                sensor_start = model.sensor_adr[cane_force_id]
                sensor_dim = model.sensor_dim[cane_force_id]
                
                forces = data.sensordata[sensor_start:sensor_start + sensor_dim]
                
                sensor_start_torque = model.sensor_adr[cane_torque_id]
                sensor_dim_torque = model.sensor_dim[cane_torque_id]
                torques = data.sensordata[sensor_start_torque:sensor_start_torque + sensor_dim_torque]
                
                print(f"t={simulation_time:.2f}s | Forces: [{forces[0]:.2f}, {forces[1]:.2f}, {forces[2]:.2f}] N "
                      f"| Torques: [{torques[0]:.2f}, {torques[1]:.2f}, {torques[2]:.2f}] N⋅m")
            
            # Update viewer
            viewer.sync()
            step_count += 1
            time.sleep(0.01)  # 10ms delay to slow down the viewer
    
    print(f"\nSimulation complete. Ran for {simulation_time:.2f} seconds.")


def analyze_forces(xml_path='go2.xml', duration=10.0):
    """
    Analyze forces without visualization for faster computation.
    """
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    cane_force_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_force')
    cane_torque_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'cane_torque')
    cane_base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cane_base')
    
    # Storage for results
    forces_log = []
    torques_log = []
    times_log = []
    
    simulation_time = 0.0
    step_count = 0
    
    print("Running force analysis...")
    
    while simulation_time < duration:
        # Apply force
        t = simulation_time
        force_x = 20.0 * np.sin(2 * np.pi * t / duration)
        force_z = -10.0 * np.cos(2 * np.pi * t / duration)
        
        data.xfrc_applied[cane_base_id, 0] = force_x
        data.xfrc_applied[cane_base_id, 2] = force_z
        
        mujoco.mj_step(model, data)
        simulation_time += model.opt.timestep
        
        # Log sensor data
        sensor_start = model.sensor_adr[cane_force_id]
        sensor_dim = model.sensor_dim[cane_force_id]
        forces = data.sensordata[sensor_start:sensor_start + sensor_dim].copy()
        
        sensor_start_torque = model.sensor_adr[cane_torque_id]
        sensor_dim_torque = model.sensor_dim[cane_torque_id]
        torques = data.sensordata[sensor_start_torque:sensor_start_torque + sensor_dim_torque].copy()
        
        forces_log.append(forces)
        torques_log.append(torques)
        times_log.append(simulation_time)
        
        if step_count % 500 == 0:
            print(f"  Processed {simulation_time:.1f}s...")
        
        step_count += 1
    
    # Analyze results
    forces_array = np.array(forces_log)
    torques_array = np.array(torques_log)
    times_array = np.array(times_log)
    
    print(f"\nForce Statistics (over {duration}s):")
    print(f"  X-Force:  mean={forces_array[:, 0].mean():.2f} N, "
          f"std={forces_array[:, 0].std():.2f} N, "
          f"max={np.abs(forces_array[:, 0]).max():.2f} N")
    print(f"  Y-Force:  mean={forces_array[:, 1].mean():.2f} N, "
          f"std={forces_array[:, 1].std():.2f} N, "
          f"max={np.abs(forces_array[:, 1]).max():.2f} N")
    print(f"  Z-Force:  mean={forces_array[:, 2].mean():.2f} N, "
          f"std={forces_array[:, 2].std():.2f} N, "
          f"max={np.abs(forces_array[:, 2]).max():.2f} N")
    
    print(f"\nTorque Statistics (over {duration}s):")
    print(f"  X-Torque: mean={torques_array[:, 0].mean():.2f} N⋅m, "
          f"std={torques_array[:, 0].std():.2f} N⋅m, "
          f"max={np.abs(torques_array[:, 0]).max():.2f} N⋅m")
    print(f"  Y-Torque: mean={torques_array[:, 1].mean():.2f} N⋅m, "
          f"std={torques_array[:, 1].std():.2f} N⋅m, "
          f"max={np.abs(torques_array[:, 1]).max():.2f} N⋅m")
    print(f"  Z-Torque: mean={torques_array[:, 2].mean():.2f} N⋅m, "
          f"std={torques_array[:, 2].std():.2f} N⋅m, "
          f"max={np.abs(torques_array[:, 2]).max():.2f} N⋅m")


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'analyze':
        # Run analysis without visualization
        analyze_forces()
    else:
        # Run interactive visualization
        main()
