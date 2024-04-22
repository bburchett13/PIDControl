stepPlot = document.getElementById('stepPlot');
let steps = 100000
let stepSize = 1;
let timeStepSize = 10/steps;
let currentTime = 0;
let time = new Array(steps);

let setpoint = new Array(steps);
let controllerOutput = new Array(steps-1);
controllerOutput[0] = 0;
controllerOutput[1] = 0;
let velocity = 0;
let bangBangcontrollerSize = 1;
let acceleration = 0;
let newControllerOutput;
// let Kp = .8;
// let Ki = 200;
// let Kd = .00005;
let Kp = 1;
let Ki = 1;
let Kd = 0;
let error = new Array(steps);
error[0] = 0;
error[1] = 0;
let cumulativeError = 0;
let dedt = 0;
let integralWindup = steps;

for (let i = 0; i < steps; i++) {

    time[i] = currentTime;
    // The commented out code is for a step input. This will later be made an available option when multiple setpoint options are made available
    if (i >= 1/timeStepSize) {

        setpoint[i] = stepSize;

    }
    else {

        setpoint[i] = 0;

    }
    // setpoint[i] = stepSize;
    //the code below this computes a sin wave setpoint. 
    // setpoint[i] = 3+Math.sin(time[i]);

    //ramp function
    // setpoint[i] = time[i];


    currentTime += timeStepSize;

    
    if (i > 1){
        controllerOutput[i] = PID(i, controllerOutput[i-1], timeStepSize, setpoint[i-1]);
        // controllerOutput[i] = bangBang(i, controllerOutput[i-1],timeStepSize,setpoint[i],bangBangcontrollerSize);

    }   

};


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

//mode: markers will give disconnected points, which might look good for the controller

	Plotly.newPlot( stepPlot, data);

// Function for Bang Bang control
function bangBang(index, previousControllerOutput, timeStep, currentFunctionOutput, controllerStrength) {

    if (index > 0) {

        if (previousControllerOutput < currentFunctionOutput) {

            acceleration = controllerStrength;
    
        }
        else {
            
            acceleration = -controllerStrength
    
        }

        velocity = velocity + acceleration * timeStepSize;
        newControllerOutput = previousControllerOutput + velocity * timeStep;

    }

    return newControllerOutput;

};

function PID(index, previousControllerOutput, timeStep, previousFunctionOutput){

    error[index] = previousFunctionOutput - previousControllerOutput;

    if (index < integralWindup){

        cumulativeError = cumulativeError + ((error[index]+error[index-1])/2)*timeStep;

    }
    else{

        cumulativeError = error.slice(index-integralWindup,index).reduce((acc, val) => acc + val);
    }
 
    dedt = (error[index] - error[index-1])/timeStep;

    // acceleration = (Kp * error[index]) + (Ki * (cumulativeError)) + (Kd * dedt);

    // velocity = velocity + (acceleration * timeStep);

    velocity = (Kp * error[index]) + (Ki * (cumulativeError)) + (Kd * dedt);

    return  (previousControllerOutput + (velocity*timeStep));
    // return previousControllerOutput + (Kp + (Ki * timeStep / 2) + (Kd/timeStep)) * error[index] + (-Kp + (Ki * timeStep/2 - (2 * Kd / timeStep))) * previousError + (Kd / timeStep) * error[index - 2];

};
