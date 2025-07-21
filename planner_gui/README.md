# Planner GUI
The planner GUI uses the optimization algorithms defined in the `global_planner` package.

Run the planner:
```bash
laraec launch localhost planner
```

## Usage
1. Launch the planner
2. Chose map in the 'map information' section
3. Choose a custom name for the raceline, like as `min_curve` or `centerline`
4. Launch the 'Global Planner' from the tools section
5. In the opened 'Global Planner' window, adjust the parameters and generate a raceline. The files will be automatically saved to the appropriate location.
6. After the raceline is defined, you can continue defining the sectors and overtaking sectors in the 'Sector Tuner' or 'OT Sector Tuner' from the tools section

Errors from the backend are currently only printend in command line. So please also check the command line if something is not working as expected.
## Author

Luis Denninger <l_denninger@uni-bonn.de>