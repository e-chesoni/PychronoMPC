# Skid Steer Vehicle Navigation Simulation

## Overview

This repository is dedicated to the simulation aspect of our project on optimizing skid steer vehicle trajectories. We have integrated PyChrono and Drake to simulate the navigation of skid steer vehicles through various terrains, providing a comprehensive and efficient approach to vehicle trajectory simulation.

## Key Components

- **PyChrono**: Acts as a Python wrapper for the Chrono C++ simulation library, offering a streamlined setup process and efficient simulation capabilities.
- **MPC_Test**: We utilize the solution from MPC_Test to run the simulation

## Simulation Workflow

1. **MPC_Test Integration**: We solve the trajectory problem using MPC_Test and port the solution into Pychrono.
2. **PyChrono Integration**: Facilitates the simulation of skid steer vehicles with a user-friendly Python interface, ensuring efficient setup and execution.
3. **PyChrono Simulation**: Integrates the solution from Drake into the PyChrono simulation environment, ensuring accurate and realistic vehicle navigation simulation.

## Usage Guidelines

- **Installation**: Ensure that both PyChrono and Drake are correctly installed and configured in your environment. 
- **Running Simulations**: Follow the provided scripts to initiate vehicle simulations. Modify parameters as needed to fit specific terrain or vehicle models.
- **Customization**: Users can customize the simulations by altering the terrain types, vehicle parameters, and control strategies.

## Note on Computational Approach

We are aware of PyChrono’s advisory against direct numerical computation on Chrono objects due to potential simulator crashes. Hence, we have strategically shifted computationally intensive tasks to Drake, ensuring stability and efficiency in our simulations.

## Contribution and Feedback

We welcome contributions and feedback to improve the simulation models and extend the capabilities of this repository. Please feel free to submit pull requests or raise issues for enhancements and bug fixes.
