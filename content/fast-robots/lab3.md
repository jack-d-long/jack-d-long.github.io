+++
title = "Lab 3"
date = "2025-02-13"

[taxonomies]

[extra]
comment = true
+++ 
# Previous: [Lab 2](/fast-robots/lab2)

# Prelab
Note the I2C sensor address: 0x69 is IMU, 0x29 is TOF

Briefly discuss the approach to using 2 ToF sensors: We don't have great sample rate per TOF, so we make up for it with more samples. 

I plan to put both TOFs on the front of my robot. While this may cause it to miss some obstacles to its left and right, I believe that these issues will be mitigated with proper angle control of the robot. When placed in an unfamiliar environment, it can simply do a 360 degree spin to map it out. I believe that interpretation of the sensor data will be easiest with them both in the same location on the robot. 

***Sketch of wiring diagram (with brief explanation if you want)***

# Lab Tasks
***Picture of your ToF sensor connected to your QWIIC breakout board***
***Screenshot of Artemis scanning for I2C device (and discussion on I2C address)***




I chose the long mode - The resolution at short distances is still sufficient to avoid collisions, but is better at long (on a room scale) distances where mapping will be more useful.

I used the shutoff pin to change the I2C address of one of the TOFs, allowing both to be read separately. 

Using two TOFs allowed me to measure differences in performance based on material measured. TOF1 in this case measured my bare hand, while TOF2 measured my matte black jacket sleeve. My hand, the more reflective material, made the TOF much more precise.  

<img src="/files/lab3/Tof2black.png" alt="TOF bottleneck"  width = 600 >

Tof sensor speed: Discussion on speed and limiting factor; include code snippet of how you do this

{% note(clickable=true,hidden = true,header = "TOF Bottleneck Test") %}

```c++

distanceSensor1.startRanging(); 
  distanceSensor2.startRanging();
  // int start_time, delay_time;
  // start_time = millis();
  if (distanceSensor1.checkForDataReady())
  {
    int distance1 = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    Serial.print("Distance1(mm): ");
    Serial.print(distance1);
    Serial.println("   ");
  }
  if (distanceSensor2.checkForDataReady())
  {
    int distance2 = distanceSensor2.getDistance(); 
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    Serial.print("Distance2(mm): ");
    Serial.print(distance2);
    Serial.println("   ");
  }


  //time delay
  Serial.print("T: ");
  Serial.print(millis());
  Serial.print("   ");

  Serial.println();

```
{% end %}

Output: 

<img src="/files/lab3/bottleneck.png" alt="TOF bottleneck"  width = 600 >

It's clear that the TOF ready time is the bottleneck in this case. New TOF values ready approximately every 50 ms. However, ranging does add a small delay to the loop in addition to the various serial prints of 1-2 ms. 



Time v Distance: Include graph of data sent over bluetooth (2 sensors)

I wrote a command *RECORD_TOF* which grabs 100 values from each TOF sensor and sends via BLE using my existing *GET_DATA* command. 

{% note(clickable=true,hidden = true,header = "RECORD_TOF") %}

```c++
case RECORD_TOF:
          tx_estring_value.clear();
          distanceSensor1.startRanging(); 
          distanceSensor2.startRanging();
          digitalWrite(blinkPin, HIGH); //record
          for (int i = 0; i < TOFStore; i++) {
            
            TOFs[i][0] = distanceSensor1.getDistance();
            TOFs[i][1] = distanceSensor2.getDistance();
            TOFs[i][2] = millis();
            delay(50);
            
          }
          digitalWrite(blinkPin, LOW); //stop recording
        break;
```

{%end%}

I generated the following values with the testing setup below:
<img src="/files/lab3/TOF6inResults.png" alt="TOF6 in results"  width = 700 >

<img src="/files/lab3/TOF6inTest.png" alt="TOF6 in results"  width = 600 >


I implemented the TOF recording into my main loop via a *record_TOF()* function which is called when either of the two TOFs are ready. 

{% note(clickable=true,hidden = true,header = "record_TOF() V1") %}

```c++

record_TOF()
{

  int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor

  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();

  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();




  TOFs[TOFStored][0] = distance1; //tof1
  TOFs[TOFStored][1] = distance2;  //tof2
  TOFs[TOFStored][2] = millis(); //toftime

  TOFStored++;


}

```
{%end%}

This significantly slowed IMU and TOF data from their standalone tests. The IMU recorded 3079 datapoints in 20 seconds, and the TOF recorded 86, for sample rates of 154Hz and 4.3Hz, respectively. The IMU sample rate is reasonable, but TOF appears too low to be useful -- We want about 10Hz from it. I collected the data below with a series of pitches, rolls, yaws, and oscillations in the z-axis (where TOFs point). 


<img src="/files/lab3/finalIMU.png" alt="IMU"  width = 1200 >

<img src="/files/lab3/finalTOF.png" alt="TOF"  width = 800 >


To try and speed up execution, I created two separate lenArrx2 arrays for storing individual TOF data and their timestamps, making my *record_TOF()* look like this: 

{% note(clickable=true,hidden = true,header = "record_TOF() V2") %}

```c++

record_TOF(int i)
{
  if(i==1){
    int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    TOF1s[TOF1Stored][0] = distance1; //tof1
    TOF1s[TOF1Stored][1] = millis(); //tof1time
    TOF1Stored++;

  }else if(i==2){
    int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    TOF2s[TOF2Stored][0] = distance2;  //tof2
    TOF2s[TOF2Stored][1] = millis(); //tof2time
    TOF2Stored++;

  }else{

    Serial.println("bad TOF index");
  }

}


```
{%end%}

THis resulted in even less recorded TOF values -- an average of 56 per TOF per 20 seconds, or 2.8 Hz. Though this seems lower than the last test, the TOFs are likely readying at the same interval and some values were just being double counted before. Still, the sample rate of the TOFs is too low.  

## Collaborations
I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively, and referenced [Daria's](https://pages.github.coecis.cornell.edu/dak267/dak267.github.io/#contact) and [Mavis'](https://mavisfu.github.io/lab3.html) site for multi-TOF operation. 

# Next: [Lab 4](/fast-robots/lab3)
