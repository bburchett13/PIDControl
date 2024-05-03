# PIDControl
This project is to make a PID control simulation


Users can select controller type (bang-bang or PID), input type (step, impulse, ramp, sine), and gains
Outputs of the system and controller are calculated for 10 seconds with a resolution of 0.0001 seconds
The system that is controlled is a simple boiler/chiller system. The system is shown below: 

    nextSystemOutput = previousSystemOutput + (1 * calculatedControllerOutput*timeStep);

    nextSystemOutput -= 0.02 * timeStep;

The controller applies some change to the system, and the temperature is lowered by a small value at every time step. 

The project uses plotly to create a plot of the data
