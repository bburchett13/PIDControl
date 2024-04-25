stepPlot = document.getElementById('stepPlot');

//Variables relating to time and number of discrete time points
let steps = 100000
let timeStepSize = 10/steps;
let currentTime = 0;
let time = new Array(steps);

//setting up arrays
let setpoint = new Array(steps);
let controllerOutput = new Array(steps-1);
controllerOutput[0] = 0;
controllerOutput[1] = 0;
let error = new Array(steps);
error[0] = 0;
error[1] = 0;

//Variables relating to controller/setpoint size
let bangBangcontrollerSize = 100;
let setpointForce;
let stepSize = 1;

//PID Variables
let cumulativeError = 0;
let dedt = 0;
let integralWindup = steps;
let Kp = 30;
let Ki = 3;
let Kd = 0.03;

//Control Loop
for (let i = 0; i < steps; i++) {

    time[i] = currentTime;
    // The commented out code is for a step input. This will later be made an available option when multiple setpoint options are made available
    if (i >= 1/timeStepSize) {

        setpointForce = stepSize;

    }
    else {

        setpointForce = 0;

    }
    setpoint[i] = setpointForce;
    //the code below this computes a sin wave setpoint. 
    // setpoint[i] = setpointForce+Math.sin(time[i]);

    //ramp function
    // setpoint[i] = time[i];
    

    currentTime += timeStepSize;

    
    if (i > 1){
        controllerOutput[i] = PID(i, controllerOutput[i-1], timeStepSize, setpoint[i-1]);
        // controllerOutput[i] = bangBang(i, controllerOutput[i-1],timeStepSize,setpoint[i],bangBangcontrollerSize);

    }   

};

//Plotting
//#region
let stepFunction = {

    x: time,

	y: setpoint,

    mode: 'scatter'

};

let controllerPlot = {

    x: time,
    y: controllerOutput,
    mode: 'scatter',
    line: {shape: 'spline', smoothing: 1.3}
};

let data = [stepFunction, controllerPlot];

let layout = {

    yaxis: {range: [0, 1.5]},
    xaxis: {range: [0, 4]}

};
//#endregion

//mode: markers will give disconnected points, which might look good for the controller

	Plotly.newPlot( stepPlot, data);

// Function for Bang Bang control
function bangBang(index, previousControllerOutput, timeStep, currentFunctionOutput, controllerStrength) {

    if (index > 0) {

        if (previousControllerOutput < currentFunctionOutput) {

            calculatedControllerOutput = controllerStrength;
    
        }
        else if (index > currentFunctionOutput) {
            
            calculatedControllerOutput = -controllerStrength
    
        }
        else {

            acceleration = 0;
        }

        newControllerOutput = system(previousControllerOutput,timeStep, calculatedControllerOutput);

    }

    return newControllerOutput;

};

//PID function
function PID(index, previousSystemOutput, timeStep, previousFunctionOutput){

    error[index] = previousFunctionOutput - previousSystemOutput;

    if (index < integralWindup){

        cumulativeError = cumulativeError + ((error[index]+error[index-1])/2)*timeStep;

    }
    else{

        cumulativeError = error.slice(index-integralWindup,index).reduce((acc, val) => acc + val);
    }
 
    dedt = (error[index] - error[index-1])/timeStep;

    calculatedControllerOutput = (Kp * error[index]) + (Ki * (cumulativeError)) + (Kd * dedt);

    newControllerOutput = system(previousSystemOutput, timeStep, calculatedControllerOutput);

    return newControllerOutput;

};


// defining a water boiler/chiller system for testing
function system(previousSystemOutput,timeStep, calculatedControllerOutput) {

    nextSystemOutput = previousSystemOutput + (1 * calculatedControllerOutput*timeStep);

    nextSystemOutput -= 0.02 * timeStep;

    return nextSystemOutput;

};