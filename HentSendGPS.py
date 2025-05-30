# ========== IMPORTS ==========
import serial      # Library for serial communication with Arduino
import time        # Library for time-related functions  
import requests    # Library for HTTP requests to GPS server

# ========== SERIAL CONFIGURATION ==========
ser = serial.Serial("/dev/serial0", 9600, timeout=1)  # Initialize serial connection on /dev/serial0 with 9600 baud
time.sleep(2)  # Wait 2 seconds for stable connection

# ========== SERVER CONFIGURATION ==========
url = "http://152.115.77.165:50412/gps-data.json"  # URL for GPS data in JSON format

try:
    # ========== GPS DATA ==========
    response = requests.get(url)                   # Send HTTP GET request to server
    
    
    if response.status_code == 200:                # If server responds successfully (status code 200)
        data = response.json()                     # Parse JSON response to Python dictionary 
        latitude = data['latitude']                # Get latitude from JSON data
        longitude = data['longitude']              # Get longitude from JSON data

        # ========== DATA FOR ARDUINO ==========
        gps_string = f"L{latitude};{longitude}\n"  # Create string in format "Llat;long\n"
        ser.write(gps_string.encode('ascii'))      # Send formatted string to Arduino via serial

        # ========== DEBUG OUTPUT ==========
        print("Sendte til Arduino:", gps_string.strip())  # Print confirmation (strip removes newline)
    else:
        print("Kunne ikke hente GPS-data. Statuskode:", response.status_code)  # Print error if request failed
except Exception as e:
    print("Fejl:", e)  # Print any other exceptions that occur
