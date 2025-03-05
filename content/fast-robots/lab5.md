+++
title = "Lab 5"
date = "2025-03-02"

[taxonomies]

[extra]
comment = true
+++ 
# Previous: [Lab 4](/fast-robots/lab4)

# Prelab

By implementing control of the Artemis over BLE via a command DRIVE_OPENLOOP, I was able to validate my motor calibration constand and deadband much more easily. 


{% note(clickable=true,hidden = true,header = "Open-Loop Drive") %}

```c++
...

case DRIVE_OPENLOOP:

         success = robot_cmd.get_next_value(PWM);
        if (!success) return;
        success = robot_cmd.get_next_value(scaleOL);
        if (!success) return;

      
        if (PWM>0 && PWM<255) {
          forward(PWM, scaleOL);
        }else if (PWM<0 && PWM>-255){
          backward(-1*PWM, scaleOL);
        }else if (PWM == 999){
          spin(150, scaleOL); 
        }else{
          stop();
        }

break; 

...

void forward(int speed, float scale) {
  analogWrite(AB1IN_LEFT,speed*scale); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,speed); 
  analogWrite(AB2IN_RIGHT,0);
}
void backward(int speed, float scale) {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,speed*scale);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,speed);
}
void stop() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,0);
}
void spin(int speed, float scale) {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,speed*scale);
  analogWrite(AB1IN_RIGHT,speed); 
  analogWrite(AB2IN_RIGHT,0);
}
```
{% end %}


I also implemented PID control of the robot via a command PID_POSITION_CONTROL, which simply loops the following section, where pid_position_tof() returns a PWM input value. 

{% note(clickable=true,hidden = true,header = "PID Loop") %}

```c++

      pid_time[i] = millis() - currentMillis;
      pid_tof[i] = distanceSensor2.getDistance();
      distanceSensor2.clearInterrupt();
      distanceSensor2.stopRanging();
      pid_speed[i] = pid_position_tof(Kp, Ki, Kd, pid_tof[i], target_tof);
      pid_p[i] = Kp * err;
      pid_i[i] = Ki * integral_err;
      pid_d[i] = Kd * err_d;
```
{% end %}


# Lab Tasks

I used the following function to implement PID control, with some logic to eliminate the deadband. : 
{% note(clickable=true,hidden = true,header = "PID Control") %}

```c++
int pid_position_tof (float Kp, float Ki, float Kd, float current_pos, float target_pos) {
  

  currentMillis = millis();
  dt = currentMillis - prev_time; 
  prev_time = currentMillis;
  err = current_pos - target_pos;
  err_d = (err - prev_err) / dt;
  

  
  // Calculate speed control signal
  int speed_control = (int)(Kp * err + Ki * err_i + Kd * err_d);
  if (speed_control > max_speed) {
    speed_control = max_speed;
  }
  if (speed_control < (-1 * max_speed)) {
    speed_control = -1 * max_speed;
  }
  if (speed_control > 0) {
    if (speed_control > deadband){
      forward(speed_control, globalscale);
    }else {
      forward(0, globalscale);
    }
    
  }
  else if (speed_control < 0) {
    if (speed_control < -1*deadband){
      backward(-1*speed_control, globalscale);
    }else {
      backward(0, globalscale);
    }
  }
  prev_err = err;
  return speed_control;

```
{% end %}


My TOF readings averaged around 9.6 Hz, an improvement on the previous lab. The system provided a PID control input at around 75 Hz, so we'll eventuallyneed some sort of extrapolation in the TOF values to take advantage of the higher-frequency control input.


To compensate for the motor deadband, I used the following logic: 
{% note(clickable=true,hidden = true,header = "Deadband Removal ") %}

```c++

  ... 
  int speed_control = (int)(Kp * err + Ki * integral_err + Kd * err_d);
  if (speed_control > max_speed) {
    speed_control = max_speed;
  }
  if (speed_control < (-1 * max_speed)) {
    speed_control = -1 * max_speed;
  }
  if (speed_control > 0) {
    if (speed_control > deadband){
      forward(speed_control, globalscale);
    }else {
      forward(deadband, globalscale);
    }
    
  }
  else if (speed_control < 0) {
    if (speed_control < -1*deadband){
      backward(-1*speed_control, globalscale);
    }else {
      backward(-1*deadband, globalscale);
    }
  }
  ...

```
{% end %}

I ran into a bit of trouble with my integrator maxing out my anti-windup logic and providing a giant impulse near the setpoint:

<img src="/files/lab5/PID_Nointerp_samegain_fixtimes.png" alt="PID_Nointerp_samegain_fixtimes"  width = 600 >



So I had to modify the max values a bit:





## Collaborations

I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively, and referenced [Daria's](https://pages.github.coecis.cornell.edu/dak267/dak267.github.io/#contact) and [Mavis'](https://mavisfu.github.io/lab3.html) site for wiring and local test code snippets.


# Next: [Lab 5](/fast-robots/lab4