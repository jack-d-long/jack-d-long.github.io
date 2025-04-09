+++
title = "Lab 8"
date = "2025-04-02"

[taxonomies]

[extra]
comment = true
+++ 
# Previous: [Lab 6](/fast-robots/lab6)

## Lab Tasks

### Kalman Filter Refinements

As a first step in speeding up my process, I implemented update logic into my Kalman filter. In its previous iteration, the filter interpreted all values in my raw TOF array as sensor measurements, even those that were reused from a previous measurement. Now, when a new TOF value is received, the `update` flag flips high and the filter performs all steps. With `update` low (i.e. at all other timesteps), the filter only performs prediction and returns predicted values. 

```c++



// state, uncertainties, measured val
KF_vars kalman_filter(Matrix<2,1> mu, Matrix<2, 2> sigma, Matrix<1,1>  u, Matrix<1,1>  y, bool update){

    // PREDICT

    mu_p = Ad * (mu) + Bd * u;
    

    sigma_p = Ad * sigma * ~Ad + sigma_u; 

    if (update){
    
    // UPDATE

    sigma_m =  (C * sigma_p * ~C) + sigma_z;
    

    sigma_m_scalar_inv = 1/sigma_m(0);

    sigma_m_inv = {sigma_m_scalar_inv};

    

    kkf_gain =  sigma_p * ~C * sigma_m_inv ; 
    

    y_m =   y -  ( C * mu_p);

    mu_out = mu_p + kkf_gain * y_m ;

    
    
    sigma_out = I  - kkf_gain * C * sigma_p ; 

    }else{

      mu_out = mu_p; // if no new value, set mu, sigma to predicted instead of updated
      sigma_out = sigma_p;
      
    }
    

    KF_vars out;



    

    out.mu = mu_out;
    out.sigma = sigma_out;

    return out;
  
}


```



### Orientation Control Refinements

To make testing easier, I needed to make orientation control to a given real-world path repeatable. To do this, I added a function which 'zeroes' the measured yaw to the current orientation of the robot. The snippet below is called every time the TOF measures data, to simplify DMP calculations.

```c++

// DMP yaw calculations

...

 if (do_zero_DMP){ // if we're zeroing the measured yaw, set an offset to current yaw
    offset = yaw;
    Serial.print("Setting offset to ");
    Serial.println(offset);

}else{
    pid_yaw_dmp[pid_c] = yaw - offset; // if we're not zeroing yaw, begin to record zeroed measurements to the array
}

```

I also added a small piece of logic to compensate for sudden jumps in angle when the robot measures yaw around 180 degrees. 

```c++

target_recip = target_orientation - (target_orientation < 0 ? -1 : 1) * 180;
if (abs(err) > 180.0) {
    err += (target_recip - pid_yaw_dmp[pid_c]) / abs(target_recip - pid_yaw_dmp[pid_c]) * 360.0;
}

```

### Open-Loop Stunt

I first tried the simplest approach I could think of -- driving the car at a wall open-loop, actively braking, and applying orientation control to turn it around, before driving back open-loop. The Python workflow looked like this: 

```python


ble.send_command(CMD.SET_PID_GAINS, "2|2|0.0001|0") # structure: 2 = orientation PID, Kp, ki, kd

ble.send_command(CMD.SET_PID_TYPE, "2") # 1 = position, 2 = orientation, 3 = both
ble.send_command(CMD.START_PID, "600|0|1") # position setpoint, orientation setpoint, record y/n

ble.send_command(CMD.DRIVE_OPENLOOP, "255|0|.5") # PWM value | record or no | timespan (s)


ble.send_command(CMD.DRIVE_OPENLOOP, "999|0|.5") # PWM value | record or no | timespan (s)

ble.send_command(CMD.START_PID, "600|180|1") # position setpoint, orientation setpoint, record y/n

time.sleep(4)

ble.send_command(CMD.DRIVE_OPENLOOP, "255|0|.5") # PWM value | record or no | timespan (s)

ble.send_command(CMD.STOP_PID, "") # stop


##

```

There were a few issues in practice, though -- my orientation control was delaying a lot, and the long settling time of the control made the return inconsistent. 

<iframe width="800" height = "500" src="https://youtube.com/embed/UYFwrHU1fdg"allowfullscreen></iframe>


### Integrating Position and Orientation PID

I tried this process again with position PID to the wall and orientation after, using the following workflow: 

```python



ble.send_command(CMD.ZERO_DMP, "")
ble.send_command(CMD.SET_PID_GAINS, "2|1|0.0001|-100") # structure: 2 = orientation PID, Kp, ki, kd
ble.send_command(CMD.SET_PID_GAINS, "1|.2|0.000002|-80") # pos gains

ble.send_command(CMD.ZERO_DMP, "") # start at 0 angle


## initial setup, let orientation cook for a sec
ble.send_command(CMD.SET_PID_TYPE, "2") # 1 = position, 2 = orientation, 3 = both
ble.send_command(CMD.START_PID, "100|0|1") # position setpoint, orientation setpoint, record y/n
time.sleep(1)
ble.send_command(CMD.STOP_PID, "") 


ble.send_command(CMD.SET_PID_TYPE, "1") # 1 = position, 2 = orientation, 3 = both
ble.send_command(CMD.START_PID, "100|180|1") # position setpoint, orientation setpoint, record y/n

time.sleep(2)

ble.send_command(CMD.SET_PID_TYPE, "2") # 1 = position, 2 = orientation, 3 = both

```

This led to an interesting issue. The DMP appeared to delay in measuring yaw for a second or so, meaning the controller wouldn't detect that its inputs were having any effect, and so would continue to apply motor input in the direction of error. This caused some serious overshoot:

<img src="/files/lab8/susdelay.png" alt="susdelya"  width = 800 >


I tried to apply Stephan's fix of slowing down the DMP output data rate (ODR) even more than it already was, and got the following response:
```c++
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok); 

```

<img src="/files/lab8/delay_after_slowdown.png" alt="sys delay afetr slowdown"  height = 500 >

Although the delay did decrease slightly, I don't think that DMP ODR is the culprit, as we can see from the steps in the yaw curve that we are retrying to sample the DMP faster than it can produce values. So I looked elsewhere. 

After checking out my mainloop, I found that I wasn't sampling from the DMP during the initial stage of position control, so the DMP buffer was filling with zeroes while the car was driving towards the wall. When I asked the IMU to begin sampling the yaw angle, it grabbed the most distant values of the FIFO buffer, resulting in a time delay. This delay caused the closed-loop orientation to become much less stable, resulting in >300% overshoot in the first case. 

To fix this, I simply had to sample from the DMP buffer during position control in my mainloop using a function `milk_dmp()`, which pulls the DMP quaternions from the buffer and stores them to an unused variable:

```c++
// In IMU header
void milk_dmp()
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
}

...

// In mainloop

if (doPID){
    
    if (doPositionPID){
    record_TOF(2); // store TOF data to global var
    pid_position_tof(); // do position PID control with recorded TOF, drive motors 
    milk_dmp(); //    NEW     - keep pulling DMP values even when not using angle data 
    }
    if ( doOrientationPID){ 
    if (pid_c < 1){ // on startup, milk the DMP for a bit to mitigate weirdness
        start_time_milk = millis();
        while(millis() - start_time_milk < 2000){
        milk_dmp();
        }
    }     
    record_IMU(); // store DMP yaw to global var
    pid_orientation_tof(); // do orientation PID with recorded yaw, drive motors

    }

    pid_c++;

}
```


 After turning the ODR back up to half speed, I was able to get the following response:


<iframe width="800" height = "500" src="https://youtube.com/embed/94ysu8LzmYM"allowfullscreen></iframe>

<img src="/files/lab8/turnaround_success.png" alt="turn around good!!"  width = 1000 >

Now my orientation control runs exactly when called! All that was left was to speed this up and increase the distance on the ground. 

### The Stunt 
After unsuccessfully trying to implement simultaneous position and orientation control, I decided to just run the two sequentially. It didn't produce a very fast response, but under a time crunch it was the best I could do. 

This was the Python workflow I used:

```python 
# ZEROOOO - adds current angle as an offset to DMP readings for easier moving around the lab

ble.send_command(CMD.ZERO_DMP, "")

ble.send_command(CMD.SET_PID_GAINS, "2|.3|0.0000|-100") # structure: 2 = orientation PID, Kp, ki, kd
ble.send_command(CMD.SET_PID_GAINS, "1|.15|0.000002|-80") # pos gains as above


ble.send_command(CMD.SET_PID_TYPE, "1") # 1 = position, 2 = orientation, 3 = both
ble.send_command(CMD.START_PID, "300|180|1") # position setpoint, orientation setpoint, record y/n

time.sleep(1) # in range of rise time for these gains

ble.send_command(CMD.SET_PID_TYPE, "2") # 1 = position, 2 = orientation, 3 = both


time.sleep(1) # in range of rise time for these gains

ble.send_command(CMD.STOP_PID, "")


ble.send_command(CMD.SET_PID_TYPE, "1") # return to position control
ble.send_command(CMD.START_PID, "300|180|1") # drive back to start

```



Although I was able to perform a few successful stunts, they were slow and inconsistent.

<iframe width="1000" height = "700" src="https://youtube.com/embed/6AhekgcmaMM"allowfullscreen></iframe>


<iframe width="1000" height = "700" src="https://youtube.com/embed/tUV7W-p9Uzk"allowfullscreen></iframe>

<iframe width="1000" height = "700" src="https://youtube.com/embed/Zk7PFMSIwVA"allowfullscreen></iframe>


Below are the position and orientation components of the stunt: 

<img src="/files/lab8/finalstunt.png" alt="turn around good!!"  width = 1000 >




### Blooper 
I was alsos able to get this blooper video:

<iframe width="1000" height = "700" src="https://youtube.com/embed/FM8RI6x_7O0"allowfullscreen></iframe>





## Collaborations

I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively. I used ChatGPT to help with curve fitting and generating nice plots. 
 


# Next: [Lab 8](/fast-robots/lab8)



