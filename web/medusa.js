// Carribbean coordinates
//var map = L.map('map').setView([22, -75], 5) 

// Florida coordinates
var map = L.map('map').setView([22.29, -80.84], 5) 
L.control.scale().addTo(map);
//map.locate({setView: true, maxZoom: 13});

var colors = ['#ffffb2','#fed976','#feb24c','#fd8d3c','#f03b20','#bd0026'];

var lastRetiredLat = 0;
var lastRetiredLon = 0;

L.esri.basemapLayer('Imagery').addTo(map);
L.esri.basemapLayer('ImageryLabels').addTo(map);

// L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
//     attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
// }).addTo(map);


// load and plot medusa drifters
function loadMedusa(){
	d3.csv('data/activeDrifters.csv', function(drifters) { 
	//http://colorbrewer2.org/#type=sequential&scheme=YlOrRd&n=6
	
 	//console.log(drifters);
 	var marker = L.marker([parseFloat(drifters.lat), parseFloat(drifters.lon)], {title: drifters.name, tz: drifters.tz}).addTo(map).on('click', onMarkerClick);
 	
	 var popupText = drifters.name+"<br>"+
 					"<embed type=\"text/html\" src='data/"+drifters.name+"/description.html'>";
 	marker.bindPopup(popupText);
	console.log('lat:', drifters.lat);
	console.log('lon:', drifters.lon);
	document.getElementById('latlon').innerHTML = 'lat:' + drifters.lat + '  lon:' + drifters.lon;
 	var fileName = 'data/' + drifters.name + '/tracks.csv';
 	console.log(fileName);
 	// load tracks
 	var latlngs = [];
 	d3.csv(fileName, function(tracks) {
 		// console.log(tracks);

 		// // this just seems wrong because it will draw line multiple times, but when outside this function, it doesn't add latlngs
 		// latlngs.push([parseFloat(tracks.lat), parseFloat(tracks.lon)]);
 		// var polyline = L.polyline(latlngs, {color: 'blue'}).addTo(map);

 		// simple color mapping
 		var minColorAmplitude = 72;
 		var maxColorAmplitude = 80;
 		var colorAmpRange = maxColorAmplitude - minColorAmplitude;
 		var colorIndex = Math.floor((parseFloat(tracks.noise20) - minColorAmplitude) * 5 / colorAmpRange);
 		if(colorIndex < 0){
 			colorIndex = 0;
 		}
 		if (colorIndex > 5){
 			colorIndex = 5;
 		}
 		var circleMarker = L.circleMarker([parseFloat(tracks.lat), parseFloat(tracks.lon)], 
 			{radius: 2, color: colors[colorIndex], fill: true, fillOpacity:1.0}).addTo(map);
 	});
 }); 
}

setInterval(loadMedusa, 120000);

loadMedusa();

// load and plot AMS stations
d3.csv('data/activeStations.csv', function(stations) { 
	//http://colorbrewer2.org/#type=sequential&scheme=YlOrRd&n=6
	var colors = ['#ffffb2','#fed976','#feb24c','#fd8d3c','#f03b20','#bd0026'];
 	//console.log(stations);
 	var marker = L.marker([parseFloat(stations.lat), parseFloat(stations.lon)], {title: stations.name, tz: stations.tz}).addTo(map).on('click', onMarkerClick);
 	//var fileName = 'data/' + stations.name + '/tracks.csv';
 	var popupText = stations.name+"<br>"+
				"<embed type=\"text/html\" src='data/"+stations.name+"/description.html'>";

 	marker.bindPopup(popupText);

 }); 

// load retired drifters once
d3.csv('data/retiredDrifters.csv', function(drifters) { 
	//http://colorbrewer2.org/#type=sequential&scheme=YlOrRd&n=6
	
 	//console.log(drifters);
 	
 	var fileName = 'data/' + drifters.name + '/tracks.csv';
 	console.log(fileName);
 	// load tracks
 	var latlngs = [];
 	
 	d3.csv(fileName, function(tracks) {
 		//console.log(tracks);

 		// simple color mapping
 		var minColorAmplitude = 75;
 		var maxColorAmplitude = 120;
 		var colorAmpRange = maxColorAmplitude - minColorAmplitude;
 		var colorIndex = Math.floor((parseFloat(tracks.noise0) - minColorAmplitude) * 5 / colorAmpRange);
 		if(colorIndex < 0){
 			colorIndex = 0;
 		}
 		if (colorIndex > 5){
 			colorIndex = 5;
 		}
 		var circleMarker = L.circleMarker([parseFloat(tracks.lat), parseFloat(tracks.lon)], 
 			{radius: 2, color: colors[colorIndex], fill: true, fillOpacity:1.0}).addTo(map);
 		lastRetiredLat = parseFloat(tracks.lat);
 		lastRetiredLon = parseFloat(tracks.lon);
 		
 	});

 	// need to delay running this because of asynchronous d3.csv
 	setTimeout(function(){
		console.log(lastRetiredLat);
	 	console.log(lastRetiredLon);
	 	var marker = L.marker([lastRetiredLat, lastRetiredLon], {title: drifters.name, tz: drifters.tz}).addTo(map).on('click', onMarkerClick);
	 	var popupText = drifters.name+"<br>"+
	 					"<embed type=\"text/html\" src='data/"+drifters.name+"/description.html'>";
	 	marker.bindPopup(popupText);
	},200);



 });

function onMarkerClick(e){
	console.log(e.sourceTarget.options.title); // name of drifter
	loadTimeSeries(e.sourceTarget.options.title, e.sourceTarget.options.tz);
	map.setView(e.target.getLatLng(),10);

}

// load and plot time series
function loadTimeSeries(drifterID, tz){
	var fileName = 'data/' + drifterID + '/tracks.csv';
	var dataset = [];
	var dataDate = [];
	var i = 0;

	Plotly.d3.csv(fileName, function(data) {
		processData(data, drifterID, tz)
	});
}

function processData(allRows, drifterID, tz){
	//   console.log(allRows);
	  var x = [], y0 = [], y1 = [], y2 = [], y3 = [], y4 = [], y5 = [], y6 = [], y7 = [], y8 = [], y9 = [], y10 = [], y11 = [], y12 = [], y13=[],y14=[],y15=[],y16=[],y17=[],y18=[],y19=[],y20=[],y21=[],y22=[],y23=[],y24=[];
	  var yVoltage = [], zAccel = [], whistles = [];
	  var Z, ylabel;
	  // hack of time zone for plotting
	  //var tzOffset = -4;
	  var tzOffset = tz;

	  var statusTitle1 = 'Whistles';
	  var statusTitle2 = '';

	  for (var i=0; i<allRows.length; i++) {
	    row = allRows[i];
	    var unixTime = parseInt(row['date']) + (tzOffset * 3600);
	    var d = new Date(unixTime * 1000);
	    var nRows = 10;
	    //d.setUTCSeconds(unixTime);
	    var formatted = d.toISOString();
	    x.push(formatted);

	    y0.push(row['noise0']);
	    y1.push(row['noise1']);
	    y2.push(row['noise2']);
	    y3.push(row['noise3']);
	    y4.push(row['noise4']);
	    y5.push(row['noise5']);
	    y6.push(row['noise6']);
	    y7.push(row['noise7']);
	    y8.push(row['noise8']);
	    y9.push(row['noise9']);
	    y10.push(row['noise10']);
	    y11.push(row['noise11']);
	    y12.push(row['noise12']);
	    y13.push(row['noise13']);
	    y14.push(row['noise14']);
	    y15.push(row['noise15']);
	    y16.push(row['noise16']);
	    y17.push(row['noise17']);
	    y18.push(row['noise18']);
	    y19.push(row['noise19']);
	    y20.push(row['noise20']);
	    y21.push(row['noise21']);
	    y22.push(row['noise22']);
	    y23.push(row['noise23']);
	    y24.push(row['noise24']);
	    if(row['noise4']===undefined) nRows=4;
	    if(row['voltage']!=undefined) {
	    	yVoltage.push(row['voltage']);
	    	statusTitle1 = 'Whistles or V';
	    }
	 	if(row['zAccel']!=undefined) {
	 		zAccel.push(row['zAccel']);
	 		statusTitle2 = 'mG';
	 	}
	 	if(row['whistles']!=undefined) whistles.push(row['whistles']);
	    //console.log(unixTime);
	  }
	 //console.log( 'X',x, 'Y1', y1, 'Y2', y2, 'Y3', y3);
	 console.log(d);


	 if (nRows==4){
		Z = [y0, y1, y2, y3];
	 	yLabel = ['172', '1000', '2000', '5000', '10000'];
	 }
	 else{
	 	Z = [y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13, y14, y15, y16, y17, y18, y19, y20, y21, y22, y23, y24];
	 	yLabel = ['47','94','140','188','234','281','328','421','515','656','843','1078','1359','1687','2156','2718','3421','4312','5437','6843','8625','10875','13687','17250','21703'];
	 }

	// var Z = [[1,10,20],[2,45,67], [54,76,12]]
	 var data = [
		 {
		 	z: Z,
		 	x: x,
		 	y: yLabel,
		 	type: 'heatmap',
		 	colorbar: {title: 'dB'},
		 }
	 ];

	 var layout = {
		title: drifterID,
		yaxis: {title: 'Frequency (Hz)'},
	 };

	 Plotly.newPlot('plot', data, layout);

	var trace1 = {
		x: x,
		y: yVoltage,
		name: 'Volts',
		yaxis: 'y1',
		type: 'line'
	};
	var trace2 = {
		x: x,
		y: zAccel,
		name: 'Acceleration (mG)',
		yaxis: 'y2',
		type: 'line'
	};
	var trace3 = {
		x: x,
		y: whistles,
		name: 'whistles',
		yaxis: 'y1',
		type: 'bar'
	};
	
	layout = {
	title: '',
	yaxis: {title: statusTitle1},
	yaxis2: {
	    title: statusTitle2,
	    overlaying: 'y',
	    side: 'right'
	  }
	};
	 var traces = [trace1, trace2, trace3];
	 Plotly.newPlot('plot2', traces, layout);

	  //makePlot(x, y1, y2, y3, y4, y5, drifterID);
	  //makePlot(x, y1, drifterID);
	}

function makePlot( x, y1, y2, y3, y4, y5, drifterID){
	var plotDiv = document.getElementById('plot');
	var trace1 = {
		x: x,
		y: y1,
		name: '.2-1 kHz',
		yaxis: 'y1',
		type: 'line'
	};
	var trace2 = {
		x: x,
		y: y2,
		name: '1-2 kHz',
		yaxis: 'y1',
		type: 'line'
	};
	var trace3 = {
		x: x,
		y: y3,
		name: '2-5 kHz',
		yaxis: 'y1',
		type: 'line'
	};
	var trace4 = {
		x: x,
		y: y4,
		name: '5-20 kHz',
		yaxis: 'y1',
		type: 'line'
	};
	var trace5 = {
		x: x,
		y: y5,
		name: 'mg',
		yaxis: 'y2',
		type: 'line'
	};

	var layout = {
	title: drifterID,
	yaxis: {title: 'dB'},
	yaxis2: {
	    title: 'Accel',
	    overlaying: 'y',
	    side: 'right'
	  }
	};

	var traces = [trace1, trace2, trace3, trace4, trace5];
	//var traces = [trace1]

	Plotly.newPlot('plot', traces, layout);
}