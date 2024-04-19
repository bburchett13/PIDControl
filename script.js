stepPlot = document.getElementById('stepPlot');
let steps = 1000
let stepSize = 1;
let currentTime = 0;
let time = new Array(steps);

let stepInput = new Array(steps);

for (let i = 0; i < steps; i++) {

    if (i >= 10) {

        stepInput[i] = stepSize;

    }
    else {

        stepInput[i] = 0;

    }

    time[i] = currentTime;
    currentTime += 0.01;

};

let stepFunction = [{

    x: time,

	y: stepInput,

    mode: 'markers'

}];

	Plotly.newPlot( stepPlot, stepFunction);