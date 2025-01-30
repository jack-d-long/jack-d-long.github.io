+++
title = "Lab 1B"

date = "2025-01-29"

[taxonomies]

[extra]
comment = true
+++
# Previous: [Lab 1A](/fast-robots/lab1a)
# Setup and Configurations

I created a Python environment and installed Jupyter Lab along with the other required packages and class codebase. I was able to start the Jupyter server successfully.

 <img src="/files/lab1/jupyter.png" alt="Picture of successful Jupyter serve"  width = 700 height = auto >

I uploaded **ble_arduino.ino** to the Artemis and was able to read its unique MAC address, as well as generate a uuid to update **connection.yaml**. 

<img src="/files/lab1/bleMAC.png" alt="ble MAC" width = 350 height = auto >

<br>
<img src="/files/lab1/uuid.png" alt="UUID" width = 350 height = auto >


# Lab

## Task 1

I sent a string value from the computer to the Artemis board using the ECHO command. The computer received and printed an augmented string.
{% note(clickable=true,hidden = true,header = "ECHO") %}

```c++
case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            
            if (!success)
                return;
            
            /*
             * Your code goes here.
             
             */
            tx_estring_value.clear();
            tx_estring_value.append("Robot says ");
            Serial.println(char_arr);
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
```

{% end %}

<br>
<br>

## Task 2

I sent three floats to the Artemis board using the *SEND_THREE_FLOATS* command and extracted the three float values in the Arduino sketch.


{% note(clickable=true,hidden = true,header = "SEND_THREE_FLOATS") %}

```c++
case SEND_THREE_FLOATS:
            float float_a, float_b, float_c;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;


            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;


            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);

            break;
```

{% end %}



<br>
<br>

## Task 3

I added a command *GET_TIME_MILLIS* which makes the robot reply write a string such as "T:123456" to the string characteristic.


{% note(clickable=true,hidden = true, header = "GET_TIME_MILLIS") %}

```c++
    case GET_TIME_MILLIS:
            // clear and update string characteristic with current time
            currentMillis = millis();
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)currentMillis);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
    break;
```

{% end %}

<br>
<br>


## Task 4

I set up a notification handler in Python to receive the string value (*BLEStringCharacteristic* in Arduino) from the Artemis board. It uses the *ble* function *start_notify()* with my *notification_handler()* to receive updates to BLE GATT characteristics concurrent with other code. 

{% note(clickable=true,hidden = true,header = "notification_handler()") %}

```python
def notification_handler(uuid, data):

    # Decode the byte data into a string

    received_string = ble.bytearray_to_string(data)
    #print(f"Received string: {received_string}")
    

    try:
        # Split the string into key-value pairs
        key_value_pairs = dict(pair.split(":") for pair in received_string.split(","))
        # Extract the time value
        time_value = key_value_pairs.get("T", "Time not found")
        # extract temp value
        temp_value = key_value_pairs.get("F", "Temp not found")
        print(f"Extracted time: {time_value}")
        print(f"Extracted temp: {temp_value}")
    except Exception as e:
        print(f"Error parsing the string: {e}")
```

{% end %}

<br>
<br>

## Task 5

I wrote a loop that gets the current time in milliseconds and sends it to my laptop to be received and processed by *notification_handler()*. After collecting these values, I found that each was transmitted approximately 60 ms after the last. This is around 17 transmissions per second, and with 13 bytes per message (for a *char* array of length 12), approximately 200 bytes per second.

<img src="/files/lab1/test_RX_speed.png" alt="test RX speed" width = 700 height = auto >


<br>
<br>

## Task 6

I then defined a global array *timeStamps* to store time stamps, and modified *GET_TIME_MILLIS* to place each time stamp into the array rather than writing each one to the string GATT characteristic. This required another global counter *stampsWritten* to ensure I didn't index a value greater than the size of *timeStamps*. Then, I added a command *SEND_TIME_DATA* which loops over *timeStamps* and sends each data point as a string to my laptop to be processed. 

{% note(clickable=true,hidden = true,header = "GET_TIME_MILLIS") %}

```c++
    case GET_TIME_MILLIS:
            //loop through timeStamps
            if (!success){
              return;
            }
              currentMillis = millis();
              if(timeStamps[19]!=0){
                stampsWritten++;
                timeStamps[stampsWritten] = (int)currentMillis;
                
              }else{
                tx_estring_value.clear();
                tx_estring_value.append("timeStamps full");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
              }
    break;
```
{% end %}

{% note(clickable=true,hidden = true,header = "SEND_TIME_DATA") %}

```c++

    case SEND_TIME_DATA:

          for (int i=0; i<20; i++){
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((int)currentMillis);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }

    break;
```
{% end %}


<br>
<br>

## Task 7

I implemented a similar assignment process for temperature readings from the Artemis using the command *SEND_TEMP_DATA*. The notification handler is able to parse both time and temprature. 


{% note(clickable=true,hidden = true,header = "SEND_TEMP_DATA") %}

```c++
case SEND_TEMP_DATA: //lowkey also records data bc i didn't want to write a separate GET_TEMP as with GET_TIME_MILLIS and SEND_TIME_DATA
    for (int i=0; i<20; i++){
    currentMillis = millis();
    timeSteps[i] = (int)currentMillis;
    temps[i] = (float)getTempDegF();
    }
    for (int i=0; i<20; i++){
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(timeSteps[i]);
    tx_estring_value.append(",");
    tx_estring_value.append("F:");
    tx_estring_value.append(temps[i]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
break;
```
{% end %}
<br>
<br>

## Task 8

Method 1 records data at a significantly lower rate, but outputs to the computer after every round of collection. This could be useful if real-time decisions must be made based on sensor readings. For an open-loop test, method 2 is much more useful. The data recorded generated has much higher resolution than that of method 1, but there is a delay between the Artemis recording data and the computer receiving it. 

With *int* timestamps and *float* temperatures, the data recorded at each step is 8 bytes. Currently defined global variables use about 30kB, so with the 354kB remaining, we can store just over **40,000 temp and timestamp sets**

<br>
<br>

# Next: [Lab 2](../lab2)




