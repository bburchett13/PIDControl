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
// let bangBangcontrollerSize = 100;
let setpointForce;
let stepSize = 1;
let calculatedControllerOutput;

//PID Variables
let cumulativeError = 0;
let dedt = 0;
let integralWindup = steps;

//Variable for Controller Type
let controllerType = 0;

let Kp = document.getElementById("Kp");
let Ki = document.getElementById("Ki");
let Kd = document.getElementById("Kd");
let bangBangControllerSize = document.getElementById("BBControllerSize");
bangBangControllerSize.disabled = true;

makePlots(time,setpoint,controllerOutput);


function setControllerType(obj) {
    // let PIDInputs = document.getElementsByClassName("PID");

    // let bangBangInputs = document.getElementsByClassName("BangBang")

    if (obj.value === '0'){
        
        Kp.disabled = false;
        Ki.disabled = false;
        Kd.disabled = false

        bangBangControllerSize.disabled = true;

        controllerType = 0;

    }
    else if (obj.value === '1') {

        Kp.disabled = true;
        Ki.disabled = true;
        Kd.disabled = true;

        bangBangControllerSize.disabled = false;
        controllerType = 1;
    };



}

//Function for control
function runSim(){
    calculatedControllerOutput = 0;
    let systemInput = document.getElementById("systemInput");
    currentTime = 0;

    //Control Loop
    for (let i = 0; i < steps; i++) {

        time[i] = currentTime;
        //Step Input
        if (systemInput.value === '0'){

            if (i >= 1/timeStepSize) {
    
                setpointForce = stepSize;
        
            }
            else {
        
                setpointForce = 0;
        
            }
            setpoint[i] = setpointForce;

        }
        else if (systemInput.value === '1'){
            setpoint[i] = 0;

            if (i > 10000 && i < 11000){
                setpoint[i] = 1

            }


        }
        //ramp function
        else if (systemInput.value === '2') {

            setpoint[i] = time[i];

        }
        //Sine Wave
        else if (systemInput.value === '3') {
            setpointForce = 1;
            setpoint[i] = setpointForce+Math.sin(time[i]);

        }
         
    
        currentTime += timeStepSize;
    
        
        if (i > 1){
            if (controllerType === 0) {

                controllerOutput[i] = PID(i, controllerOutput[i-1], timeStepSize, setpoint[i-1], Kp.value, Ki.value, Kd.value);


            }
            else if (controllerType === 1) {

                controllerOutput[i] = bangBang(i, controllerOutput[i-1],timeStepSize,setpoint[i],bangBangControllerSize.value);

            };
        };   
    };

    makePlots(time, setpoint, controllerOutput);

    
};

//Plotting
function makePlots(time, setpoint, controllerOutput) {

    //#region
    let stepFunction = {

        x: time,

        y: setpoint,

        mode: 'scatter',

        name:"Signal"

    };

    let controllerPlot = {

        x: time,
        y: controllerOutput,
        mode: 'scatter',
        line: {shape: 'spline', smoothing: 1.3},
        name: "Controller"
    };

    let data = [stepFunction, controllerPlot];

    let layout = {

        yaxis: {range: [0, 1.5]},
        xaxis: {range: [0, 4]}

    };
    //#endregion

    //mode: markers will give disconnected points, which might look good for the controller

    Plotly.newPlot( stepPlot, data);


}


// Function for Bang Bang control
function bangBang(index, previousControllerOutput, timeStep, currentFunctionOutput, controllerStrength) {

    if (index > 0) {
        if (index % 500 === 0) {

            if (previousControllerOutput <= currentFunctionOutput) {

                calculatedControllerOutput = controllerStrength;
        
            }
            else if (index > currentFunctionOutput) {
                
                calculatedControllerOutput = -controllerStrength
        
            }

        }
            
        newControllerOutput = system(previousControllerOutput,timeStep, calculatedControllerOutput);

    }

    return newControllerOutput;

};

//PID function
function PID(index, previousSystemOutput, timeStep, previousFunctionOutput, Kp, Ki, Kd){

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