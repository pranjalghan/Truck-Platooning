# Truck Platooning Simulation

This repository contains MATLAB simulations for various truck platooning scenarios, including platoon formation, lane changing, emergency braking, and vehicle cut-in maneuvers. The simulations demonstrate the use of automated control systems to manage a platoon of trucks traveling at highway speeds with coordinated safety responses to dynamic traffic conditions.

## Description

The simulation models the behavior of a convoy of four trucks equipped with advanced driver-assistance systems (ADAS) capable of autonomous driving tasks such as:

- **Maintaining Formation**: Trucks maintain a specific formation using cruise control and minimal safety distances.
- **Emergency Braking**: Coordinated response to sudden stops by vehicles ahead to prevent collisions.
- **Lane Changing**: Executing lane change maneuvers while maintaining platoon integrity.
- **Vehicle Cut-in**: Handling scenarios where an external vehicle cuts in front of a truck within the platoon.

These scenarios are crucial for testing the reliability and effectiveness of collision avoidance and platoon management systems under controlled conditions.

## Files

The repository includes the following MATLAB scripts:

- `truck_platoon_formation.m`: Manages the basic formation of the truck platoon.
- `truck_platoon_lane_change.m`: Simulates lane changing maneuvers.
- `truck_platoon_cutin.m`: Handles scenarios where another vehicle cuts into the platoon.
- `truck_platoon_braking_system.m`: Manages emergency braking responses.

## Usage

To run these simulations, you will need MATLAB installed on your computer. Each script can be executed independently to observe different aspects of platoon dynamics:

1. Open MATLAB.
2. Navigate to the directory containing the scripts.
3. Open the desired script file.
4. Run the script by pressing the Run button or using the command window.

## Visualization

Each script generates visual outputs that represent the dynamic positioning and motion of the trucks and any other vehicles involved in the scenario. The visualization helps in understanding the response of the platoon to various simulated traffic conditions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any bug fixes, enhancements, or suggestions.

## Contact

For any questions or issues, please contact Pranjal Ghan at pranjalghan100@gmail.com.

