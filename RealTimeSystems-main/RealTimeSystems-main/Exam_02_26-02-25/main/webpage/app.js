/**
 * Add gobals here
 */

let otaTimerVar =  null;
let wifiConnectInterval = null;
let intensity_red = 100;
let intensity_green = 100;
let intensity_blue = 100;

/**
 * Initialize functions here.
 */
$(document).ready(function(){
	//getUpdateStatus();
	startDHTSensorInterval();
	$("#connect_wifi").on("click", function(){
		checkCredentials();
	}); 
	startpotenciometerInterval();
	$("#connect_wifi").on("click", function(){
		checkCredentials();
	}); 
});   

/**
 * Gets file name and size for display on the web page.
 */        
function getFileInfo() 
{
    var x = document.getElementById("selected_file");
    var file = x.files[0];

    document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
}

/**
 * Handles the firmware update.
 */
function updateFirmware() 
{
    // Form Data
    var formData = new FormData();
    var fileSelect = document.getElementById("selected_file");
    
    if (fileSelect.files && fileSelect.files.length == 1) 
	{
        var file = fileSelect.files[0];
        formData.set("file", file, file.name);
        document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";

        // Http Request
        var request = new XMLHttpRequest();

        request.upload.addEventListener("progress", updateProgress);
        request.open('POST', "/OTAupdate");
        request.responseType = "blob";
        request.send(formData);
    } 
	else 
	{
        window.alert('Select A File First')
    }
}

/**
 * Progress on transfers from the server to the client (downloads).
 */
function updateProgress(oEvent) 
{
    if (oEvent.lengthComputable) 
	{
        getUpdateStatus();
    } 
	else 
	{
        window.alert('total size is unknown')
    }
}

/**
 * Posts the firmware udpate status.
 */
function getUpdateStatus() 
{
    var xhr = new XMLHttpRequest();
    var requestURL = "/OTAstatus";
    xhr.open('POST', requestURL, false);
    xhr.send('ota_update_status');

    if (xhr.readyState == 4 && xhr.status == 200) 
	{		
        var response = JSON.parse(xhr.responseText);
						
	 	document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time

		// If flashing was complete it will return a 1, else -1
		// A return of 0 is just for information on the Latest Firmware request
        if (response.ota_update_status == 1) 
		{
    		// Set the countdown timer time
            seconds = 10;
            // Start the countdown timer
            otaRebootTimer();
        } 
        else if (response.ota_update_status == -1)
		{
            document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
        }
    }
}

/**
 * Displays the reboot countdown.
 */
function otaRebootTimer() 
{	
    document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. This page will close shortly, Rebooting in: " + seconds;

    if (--seconds == 0) 
	{
        clearTimeout(otaTimerVar);
        window.location.reload();
    } 
	else 
	{
        otaTimerVar = setTimeout(otaRebootTimer, 1000);
    }
}

/**
 * Gets DHT22 sensor temperature and humidity values for display on the web page.
 */
function getDHTSensorValues()
{
	$.getJSON('/dhtSensor.json', function(data) {
		$("#temperature_reading").text(data["temp"]);
	});
}
function getpotenciometerValues()
{
	$.getJSON('/potenciometro.json', function(data) {
		$("#potenciometro_reading").text(data["pot"]);
	});
}
/**
 * Sets the interval for getting the updated DHT22 sensor values.
 */
function startDHTSensorInterval()
{
	setInterval(getDHTSensorValues, 1500);    
}
function startpotenciometerInterval()
{
	setInterval(getpotenciometerValues, 1500);    
}


/**
 * Clears the connection status interval.
 */
function stopWifiConnectStatusInterval()
{
	if (wifiConnectInterval != null)
	{
		clearInterval(wifiConnectInterval);
		wifiConnectInterval = null;
	}
}

/**
 * Gets the WiFi connection status.
 */
function getWifiConnectStatus()
{
	var xhr = new XMLHttpRequest();
	var requestURL = "/wifiConnectStatus";
	xhr.open('POST', requestURL, false);
	xhr.send('wifi_connect_status');
	
	if (xhr.readyState == 4 && xhr.status == 200)
	{
		var response = JSON.parse(xhr.responseText);
		
		document.getElementById("wifi_connect_status").innerHTML = "Connecting...";
		
		if (response.wifi_connect_status == 2)
		{
			document.getElementById("wifi_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect. Please check your AP credentials and compatibility</h4>";
			stopWifiConnectStatusInterval();
		}
		else if (response.wifi_connect_status == 3)
		{
			document.getElementById("wifi_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
			stopWifiConnectStatusInterval();
		}
	}
}

/**
 * Starts the interval for checking the connection status.
 */
function startWifiConnectStatusInterval()
{
	wifiConnectInterval = setInterval(getWifiConnectStatus, 2800);
}

/**
 * Connect WiFi function called using the SSID and password entered into the text fields.
 */
function connectWifi()
{
	// Get the SSID and password
	/*selectedSSID = $("#connect_ssid").val();
	pwd = $("#connect_pass").val();
	
	$.ajax({
		url: '/wifiConnect.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		headers: {'my-connect-ssid': selectedSSID, 'my-connect-pwd': pwd},
		data: {'timestamp': Date.now()}
	});
	*/
	selectedSSID = $("#connect_ssid").val();
	pwd = $("#connect_pass").val();
	
	// Create an object to hold the data to be sent in the request body
	var requestData = {
	  'selectedSSID': selectedSSID,
	  'pwd': pwd,
	
	};
	
	// Serialize the data object to JSON
	var requestDataJSON = JSON.stringify(requestData);
	
	$.ajax({
	  url: '/wifiConnect.json',
	  dataType: 'json',
	  method: 'POST',
	  cache: false,
	  data: requestDataJSON, // Send the JSON data in the request body
	  contentType: 'application/json', // Set the content type to JSON
	  success: function(response) {
		// Handle the success response from the server
		console.log(response);
	  },
	  error: function(xhr, status, error) {
		// Handle errors
		console.error(xhr.responseText);
	  }
	});


	//startWifiConnectStatusInterval();
}

/**
 * Checks credentials on connect_wifi button click.
 */
function checkCredentials()
{
	errorList = "";
	credsOk = true;
	
	selectedSSID = $("#connect_ssid").val();
	pwd = $("#connect_pass").val();
	
	if (selectedSSID == "")
	{
		errorList += "<h4 class='rd'>SSID cannot be empty!</h4>";
		credsOk = false;
	}
	if (pwd == "")
	{
		errorList += "<h4 class='rd'>Password cannot be empty!</h4>";
		credsOk = false;
	}
	
	if (credsOk == false)
	{
		$("#wifi_connect_credentials_errors").html(errorList);
	}
	else
	{
		$("#wifi_connect_credentials_errors").html("");
		connectWifi();    
	}
}

/**
 * Shows the WiFi password if the box is checked.
 */
function showPassword()
{
	var x = document.getElementById("connect_pass");
	if (x.type === "password")
	{
		x.type = "text";
	}
	else
	{
		x.type = "password";
	}
}


function updateValue(val) {
	document.getElementById('rangeValue').innerText = val;
}
function updateValueintesity(val) {
	document.getElementById('intensityValue').innerText = val;
}
function updateValueMAX(val) {
	document.getElementById('rangeValueMAX').innerText = val;
}
function updateValueBuzzer(val) {
	document.getElementById('rangeValueBuzzer').innerText = val;
}
function updateValueBuzzerRangemin(val){
	document.getElementById('rangeValueBuzzerSpanmin').innerText = val;
}
function updateValueBuzzerRangemax(val){
	document.getElementById('rangeValueBuzzerSpanmax').innerText = val;
}
function updateRangeActivationFanInputMin(val){
	document.getElementById('rangeActivationFanSpanMin').innerText = val;
}
function updateRangeActivationFanInputMax(val){
	document.getElementById('rangeActivationFanSpanMax').innerText = val;
}


function toogle_led() 
{	
	$.ajax({
		url: '/toogle_led.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
	});

}

function brigthness_up() 
{	
	$.ajax({
		url: '/toogle_led.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
	});
	
}


//myfn

function rangeActivationFanbtn(){

	let valueRangeMin = document.getElementById('rangeActivationFanInputMin').value;
	let valueRangeMax = document.getElementById('rangeActivationFanInputMax').value;
	let requestData = {
		"rangeActivationFanMin":valueRangeMin,
		"rangeActivationFanMax":valueRangeMax,
	};
	
	
		  console.log(requestData);
	
		  let requestDataJSON = JSON.stringify(requestData);
	  
		  $.ajax({
			  url: '/Range_Activation_Fun.json',
			  dataType: 'json',
			  method: 'POST',
			  cache: false,
			  data: requestDataJSON, // Send the JSON data in the request body
			  contentType: 'application/json', // Set the content type to JSON
			  success: function(response) {
				// Handle the success response from the server
				console.log(response);
			  },
			  error: function(xhr, status, error) {
				// Handle errors
				console.error(xhr.responseText);
			  }
			});


}



function updateRangeActivationbtn(){
		let valueRangeMin = document.getElementById('rangeactivationbuzzerinputmin').value;
		let valueRangeMax = document.getElementById('rangeactivationbuzzerinputmax').value;
		let requestData = {
			"minRangeActivationBuzzer":valueRangeMin,
			"maxRangeActivationBuzzer":valueRangeMax,
		};
		
		
			  console.log(requestData);
		
			  let requestDataJSON = JSON.stringify(requestData);
		  
			  $.ajax({
				  url: '/Range_Activation_Buzzer.json',
				  dataType: 'json',
				  method: 'POST',
				  cache: false,
				  data: requestDataJSON, // Send the JSON data in the request body
				  contentType: 'application/json', // Set the content type to JSON
				  success: function(response) {
					// Handle the success response from the server
					console.log(response);
				  },
				  error: function(xhr, status, error) {
					// Handle errors
					console.error(xhr.responseText);
				  }
				});
	
}
		
	




document.querySelectorAll('.exclusive').forEach(checkbox => {
	checkbox.addEventListener('change', () => {
		if (checkbox.checked) {
			document.querySelectorAll('.exclusive').forEach(item => {
				if (item !== checkbox) {
					item.checked = false;
                }
            });
        }
    });
});
function updateINTColor(){
	let valueRangeMin = document.getElementById('intensity_input').value;
	const rangeCheckColor = document.querySelector('.rangeCheckBox:checked');
	let requestData = {};
	if(rangeCheckColor){
		
		switch (rangeCheckColor.value) {
			case "red":
				 requestData = {
					'color':"red",
					"valueRangeMin" : valueRangeMin,
				};
				
			  break;
			case "blue":
				 requestData = {
					'color':"blue",
					"valueRangeMin" : valueRangeMin,
				};
			  break;
			case "green":
				 requestData = {
					'color':"green",
					"valueRangeMin" : valueRangeMin,
				};
			  
			  break;
			
	
			default:
				 requestData = {
				};
			  break;
		  }

	
		  console.log(requestData);
	
		  let requestDataJSON = JSON.stringify(requestData);
	  
		  $.ajax({
			  url: '/updateINTColor.json',
			  dataType: 'json',
			  method: 'POST',
			  cache: false,
			  data: requestDataJSON, // Send the JSON data in the request body
			  contentType: 'application/json', // Set the content type to JSON
			  success: function(response) {
				// Handle the success response from the server
				console.log(response);
			  },
			  error: function(xhr, status, error) {
				// Handle errors
				console.error(xhr.responseText);
			  }
			});

	}
	else{
		console.log("ningun checkbox seleccionado");
	}

}

function updateValueBuzzerbtn(){
	let valuerangebuzzer = document.getElementById("rangeUserBuzzer").value; 
	let requestData = {
		"ValueBuzzer":valuerangebuzzer,
	};
	console.log(requestData);
	var requestDataJSON = JSON.stringify(requestData);
	
		$.ajax({
			url: '/update_buzzer_value.json',
			dataType: 'json',
			method: 'POST',
			cache: false,
			data: requestDataJSON, // Send the JSON data in the request body
			contentType: 'application/json', // Set the content type to JSON
			success: function(response) {
			  // Handle the success response from the server
			  console.log(response);
			},
			error: function(xhr, status, error) {
			  // Handle errors
			  console.error(xhr.responseText);
			}
		  });

}



function Intensity_rgb( inc )
{
	const checkedCheckbox = document.querySelector('.exclusive:checked');

	let requestData = {}
    if (checkedCheckbox) {
        console.log(`Checkbox seleccionado: ${checkedCheckbox.value}`);
		
		switch (checkedCheckbox.value) {
			case "red":
				intensity_red += inc
				if (((intensity_red)<256) && (intensity_red)>0)  {

					intensity_red =intensity_red+ inc
					requestData = {
					"color":"red",
					"intensity":`${intensity_red}`,

				};
				}
				else{
					intensity_red = intensity_red - inc
				}

				
			  break;
			case "blue":
				intensity_blue += inc
			if (((intensity_blue)<256) && (intensity_blue)>0)  {

				intensity_blue += inc
				requestData = {
				"color":"blue",
				"intensity":`${intensity_blue}`,
			};
			}
			else{
				intensity_blue = intensity_blue - inc
			}

			  break;
			case "green":
				intensity_green += inc
				if (((intensity_green)<256) && (intensity_green)>0)  {

					intensity_green += inc
					requestData = {
					"color":"green",
					"intensity":`${intensity_green}`,
	
				};
			}
			else{
				intensity_green = intensity_green - inc
			}
			  break;
			
	
			default:
			  break;
		  }
		  
		if (Object.keys(requestData).length !== 0) {

			let requestDataJSON = JSON.stringify(requestData);
			console.log(requestData);
			console.log(requestDataJSON);
		$.ajax({
			url: '/intensity_rgb_up.json',
			dataType: 'json',
			method: 'POST',
			cache: false,
			data: requestDataJSON, // Send the JSON data in the request body
			contentType: 'application/json', // Set the content type to JSON
			success: function(response) {
			  // Handle the success response from the server
			  console.log(response);
			},
			error: function(xhr, status, error) {
			  // Handle errors
			  console.error(xhr.responseText);
			}
		  });
			
		}else{
			console.log("error");
		}
		

    } else {
        console.log('Ningún checkbox seleccionado.');
    }

}

function thebigsave(){
	$.ajax({
		url: '/thebigsave.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
	});
}
/*
// Funcionalidad de los botones
document.getElementById('up').addEventListener('click', () => {
	const checkedCheckbox = document.querySelector('.exclusive:checked');
	requestData = {}
    if (checkedCheckbox) {
        //console.log(`Checkbox seleccionado: ${checkedCheckbox.value}`);
		
		switch (checkedCheckbox.value) {
			case "red":
				 requestData = {
					"color":"red",

				};
			  break;
			case "blue":
				 requestData = {
					"color":"blue",
				};
			  break;
			case "green":
				requestData = {
					"color":"green",
				};
			  
			  break;
			
	
			default:
			  break;
		  }
	
	
		var requestDataJSON = JSON.stringify(requestData);
	
		$.ajax({
			url: '/intensity_rgb_up.json',
			dataType: 'json',
			method: 'POST',
			cache: false,
			data: requestDataJSON, // Send the JSON data in the request body
			contentType: 'application/json', // Set the content type to JSON
			success: function(response) {
			  // Handle the success response from the server
			  console.log(response);
			},
			error: function(xhr, status, error) {
			  // Handle errors
			  console.error(xhr.responseText);
			}
		  });

    } else {
        console.log('Ningún checkbox seleccionado.');
    }



});

document.getElementById('down').addEventListener('click', () => {
    const checkedCheckbox = document.querySelector('.exclusive:checked');
	let cvalue;
    if (checkedCheckbox) {
        //console.log(`Checkbox seleccionado: ${checkedCheckbox.value}`);
		switch (checkedCheckbox.value) {
			case "red":
				cvalue = 3;
				console.log(`RED ${cvalue}`);
			  break;
			case "blue":
				cvalue = 4;
				console.log(`BLUE ${cvalue}`);
			  break;
			case "green":
				cvalue  = 5;
				console.log(`GREEN ${cvalue}`);
			  
			  break;
			
	
			default:
			  break;
		  }
		  
	var requestData = {
        "cvalue":cvalue
    };

	var requestDataJSON = JSON.stringify(requestData);

	$.ajax({
		url: '/intensity_rgb_down.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		data: requestDataJSON, // Send the JSON data in the request body
		contentType: 'application/json', // Set the content type to JSON
		success: function(response) {
		  // Handle the success response from the server
		  console.log(response);
		},
		error: function(xhr, status, error) {
		  // Handle errors
		  console.error(xhr.responseText);
		}
	  });

    } else {
        console.log('Ningún checkbox seleccionado.');
    }
});


*/
function updateRangeColor(){
	let valueRangeMin = document.getElementById('rangeUserMin').value;
	let valueRangeMax = document.getElementById('rangeUserMax').value;
	const rangeCheckColor = document.querySelector('.rangeCheckBox:checked');
	
	let requestData = {};
	if(rangeCheckColor){
		
		switch (rangeCheckColor.value) {
			case "red":
				 requestData = {
					'color':"red",
					"valueRangeMin" : valueRangeMin,
					"valueRangeMax" : valueRangeMax,
				};
				
			  break;
			case "blue":
				 requestData = {
					'color':"blue",
					"valueRangeMin" : valueRangeMin,
					"valueRangeMax" : valueRangeMax,
				};
			  break;
			case "green":
				 requestData = {
					'color':"green",
					"valueRangeMin" : valueRangeMin,
					"valueRangeMax" : valueRangeMax,
				};
			  
			  break;
			
	
			default:
				 requestData = {
				};
			  break;
		  }

	
		  console.log(requestData);
	
		  let requestDataJSON = JSON.stringify(requestData);
	  
		  $.ajax({
			  url: '/updateRangeColor.json',
			  dataType: 'json',
			  method: 'POST',
			  cache: false,
			  data: requestDataJSON, // Send the JSON data in the request body
			  contentType: 'application/json', // Set the content type to JSON
			  success: function(response) {
				// Handle the success response from the server
				console.log(response);
			  },
			  error: function(xhr, status, error) {
				// Handle errors
				console.error(xhr.responseText);
			  }
			});

	}
	else{
		console.log("ningun checkbox seleccionado");
	}

}






    


