+++
title = "Lab 2"
date = "2025-02-07"

[taxonomies]

[extra]
comment = true
+++
# Previous: [Lab 1B](/fast-robots/lab1b)
# Lab Tasks
## Set up the IMU
<br>
<img src="/files/lab2/IMUsetup.jpg" alt="Artemis and IMU"  width = 400 >

The serial output of the example code given is as shown:
<iframe width="450" height="315" src="https://youtube.com/embed/ytyu0kU6aGI"allowfullscreen></iframe>

We set AD0_VAL to 1 to define the I2C address of the IMU. In this case, it maintains the default address of 0x12. 

## Accelerometer
Image of output at {-90, 0, 90} degrees for pitch and roll (include equations)
# GET BETTER DATA

<br>
<img src="/files/lab2/AccelerometerPitchRollTime.png" alt="Initial accelerometer data"  width = 400 >
<iframe width="450" height="315" src="https://youtube.com/embed/DPVjiCiOTgU"allowfullscreen></iframe>
{% note(clickable=true,hidden = true,header = "Pitch and Roll") %}

```c++
    pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
    roll_a  = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
```

{% end%}

The accelerometer is very noisy, and become less accurate as pitch and roll tend to 90 degrees. This is because when either the pitch or roll axes approach the gravitational acceleration vector, the accelerometer is no longer able to read rotation data about that axis. I found the following noise spectrum when running the car nearby the IMU. 

<img src="/files/lab2/fftGood.png" alt="Initial accelerometer data"  width = 600 >

With strong noise peaks above 10 Hz, especially in pitch, a cutoff frequency of 10 Hz seemed reasonable for my calculation of $\alpha$. We first found $RC$ and substituted to find $\alpha$ using the equations
$$
f_c = \frac{1}{2\pi RC}
$$
$$
\alpha = \frac{T_s}{T_s + RC}
$$

Where $T_s$ is the sampling period of the IMU, given in seconds. For the given noise spectrum, I calculated $\alpha = .077$, producing the following low-pass-filtered car noise and ringdown readings. 

<img src="/files/lab2/FFTLPF.png" alt="FFT LPF car noise"  width = 800 >
 <img src="/files/lab2/lpfringdown.png" alt="FFT LPF ringdown"  width = 800 >




## Gyroscope
The accelerometer roughly tracked the changes in pitch, roll, and yaw expected from movement of the IMU and accelerometer readings, but struggled with the angle offset due to lack of initial conditions in the integrator. I also saw significant integrated error, especially in yaw. When the Artemis was kept static about the accelerometer and gyroscope z axis, yaw appeared to vary linearly. 

<img src="/files/lab2/gyroInttro.png" alt="Gyro unfilterd"  width = 800 >

With the complementary filter in place (in this case with $\alpha=.08$, similar to the pitch and roll LPF), pitch and roll values appeared to track the actual motion of the IMU much more closely, and lacked the high-frequency noise present even in the LPF accelerometer readings. 


{% note(clickable=true,hidden = true,header = "Complementary Filter") %}

```c++

const float alpha_gyro = .08;
dt = (micros()-last_time)/1000000.;
last_time = micros(); 
gyroFiltered[valsStored][0] = (gyroFiltered[valsStored - 1][0] + gyrosPitchRollYaws[valsStored][0] * dt) * (1 - alpha_gyro) + (LPFPitchRolls[valsStored][1] * alpha_gyro); // pitch 
gyroFiltered[valsStored][1]  = (gyroFiltered[valsStored - 1][1]  + gyrosPitchRollYaws[valsStored][1]  * dt) * (1 - alpha_gyro) + (LPFPitchRolls[valsStored][0] * alpha_gyro); // roll
gyroFiltered[valsStored][2]   = gyrosPitchRollYaws[valsStored][2];
```

{% end %}



<img src="/files/lab2/compfilter.png" alt="Gyro fitered"  width = 800 >

Yaw remained unfiltered as there is no accelerometer data to complement it. 








## Sample Data
With serial monitor prints removed, the IMU sampled at approximately 320 Hz. I didn't find the GET_IMU_DATA loop to be running any faster -- *myICM.dataReady()* never returned false.

I stored sample data in arrays and then sent to my laptop all at once, as in Lab 1. I got about 6.5 seconds of data with a 2048-element array, stored like so: 

{% note(clickable=true,hidden = true,header = "GET_IMU_DATA") %}

```c++

int storeThisMuch = 2048;
case GET_IMU_DATA:
        digitalWrite(blinkPin, HIGH);// visual cue for start data collection

        while(valsStored<storeThisMuch){
            looped++;
            if(myICM.dataReady())
            { 
            myICM.getAGMT();

            // LPF

            const float alpha = 0.08; //for now same for pitch, roll
            pitch_a_LPF[n] = alpha*pitch_a + (1-alpha)*pitch_a_LPF[n-1];
            pitch_a_LPF[n-1] = pitch_a_LPF[n];
            // Serial.print(", pitch_LPF:");
            // Serial.println(pitch_a_LPF[n]);
            LPFPitchRolls[valsStored][0] = pitch_a_LPF[n];
            roll_a_LPF[n] = alpha*roll_a + (1-alpha)*roll_a_LPF[n-1];
            roll_a_LPF[n-1] = roll_a_LPF[n];
            LPFPitchRolls[valsStored][1] = roll_a_LPF[n];

            // end LPF


            //  gyro

            dt = (micros()-last_time)/1000000.;
            last_time = micros();
            pitch_g = pitch_g + myICM.gyrX()*dt;
            roll_g = roll_g + myICM.gyrY()*dt;
            yaw_g = yaw_g + myICM.gyrZ()*dt;

            gyrosPitchRollYaws[valsStored][0] = pitch_g;
            gyrosPitchRollYaws[valsStored][1] = roll_g;
            gyrosPitchRollYaws[valsStored][2] = yaw_g; //[pitch, roll, yaw]


            // Filter 
            const float alpha_gyro = .08;
            dt = (micros()-last_time)/1000000.;
            last_time = micros(); 
            gyroFiltered[valsStored][0] = (gyroFiltered[valsStored - 1][0] + gyrosPitchRollYaws[valsStored][0] * dt) * (1 - alpha_gyro) + (LPFPitchRolls[valsStored][1] * alpha_gyro); // pitch 
            gyroFiltered[valsStored][1]  = (gyroFiltered[valsStored - 1][1]  + gyrosPitchRollYaws[valsStored][1]  * dt) * (1 - alpha_gyro) + (LPFPitchRolls[valsStored][0] * alpha_gyro); // roll
            gyroFiltered[valsStored][2]   = gyrosPitchRollYaws[valsStored][2];


            // end filter 

            // end gyro

            pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
            roll_a  = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
            pitchRollsTimes[valsStored][0] = pitch_a;
            pitchRollsTimes[valsStored][1] = roll_a;
            currentMillis = millis();
            timeStamps[valsStored] = currentMillis;
            valsStored++;
            }//endif
            
        }//end while
        digitalWrite(blinkPin, LOW); // visual cue for stop data collection

        // write and send to notif handler
        for(int i=0;i<storeThisMuch;i++){

            tx_estring_value.clear();

            tx_estring_value.append("P:");
            tx_estring_value.append(pitchRollsTimes[i][0]);
            tx_estring_value.append(",");
            tx_estring_value.append("R:");
            tx_estring_value.append(pitchRollsTimes[i][1]);
            tx_estring_value.append(",");
            tx_estring_value.append("T:");
            tx_estring_value.append(timeStamps[i]);
            //LPFs
            tx_estring_value.append(",");
            tx_estring_value.append("PL:");
            tx_estring_value.append(LPFPitchRolls[i][0]);
            tx_estring_value.append(",");
            tx_estring_value.append("RL:");
            tx_estring_value.append(LPFPitchRolls[i][1]);

            //gyros
            tx_estring_value.append(",");
            tx_estring_value.append("PG:");
            tx_estring_value.append(gyrosPitchRollYaws[i][0]);
            tx_estring_value.append(",");
            tx_estring_value.append("RG:");
            tx_estring_value.append(gyrosPitchRollYaws[i][1]);
            tx_estring_value.append(",");
            tx_estring_value.append("YG:");
            tx_estring_value.append(gyrosPitchRollYaws[i][2]);
            //  filtered
            tx_estring_value.append(",");
            tx_estring_value.append("PGF:");
            tx_estring_value.append(gyroFiltered[i][0]);
            tx_estring_value.append(",");
            tx_estring_value.append("RGF:");
            tx_estring_value.append(gyroFiltered[i][1]);
            tx_estring_value.append(",");
            tx_estring_value.append("YGF:");
            tx_estring_value.append(gyroFiltered[i][2]);

            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }


            
    break;
```
{% end %}


The above plots show well over 5 seconds of data transmitted by BLE. 

## Record a Stunt

<iframe width="450" height="315" src="https://youtube.com/embed/TLcFdr-ZEDI"allowfullscreen></iframe>

Any fine control of the car appeared impossible -- I was only able to signal movements at full speed, leading to chronic oversteer and instability when control inputs were held for too long. 

## Collaborations
I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively, and referenced [Daria's site](https://pages.github.coecis.cornell.edu/dak267/dak267.github.io/#contact) to troubleshoot FFT code. 
I used ChatGPT to generate a faster Python notification handler with less dictionary operations, for plotting syntax, and for more information on FFT implementation. 

# Next: [Lab 3](/fast-robots/lab3)
