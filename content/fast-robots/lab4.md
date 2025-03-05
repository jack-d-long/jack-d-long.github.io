+++
title = "Lab 4"
date = "2025-03-02"

[taxonomies]

[extra]
comment = true
+++ 
# Previous: [Lab 3](/fast-robots/lab3)

# Prelab




I decided to connect my motor drivers to Artemis pins A13, A14, A15, and A16, and wire the car like so: 

<img src="/files/lab/wired.jpg" alt="Wiring diagram"  width = 600 >

We power the motors with a separate battery from the Artemis to reduce the effect of back-EMF on the Artemis. There's no reason to have the varying load of the motors effect the stability of the Artemis' computations. Two smaller batteries in separate locations also helps with reducing the form factor of the whole car. 



# Lab Tasks

After soldering my motor drivers, I passed a PWM value of 200, leading to the following duty cycle when supplied with 3.7V from the lab power supply. 

<iframe width="600" height = "400" src="https://youtube.com/embed/O-vv-xoWVM8"allowfullscreen></iframe>

I chose to set the supply for 3.7 V because the 1S LiPos we use operate at the same voltage. I set a maximum current draw of 1.5A, well below the limits of the parallel motor driver, for safety. 

To initially test the drivers, I wrote functions to move the car forward, backward, and spin it. 

{% note(clickable=true,hidden = true,header = "Motor Test") %}

```c++
#define AB1IN_LEFT 13
#define AB2IN_LEFT 14
#define AB1IN_RIGHT 16
#define AB2IN_RIGHT 15

#define SCALEFACTOR 1.2
void setup() {
  pinMode(AB1IN_LEFT,OUTPUT);
  pinMode(AB2IN_LEFT,OUTPUT);
  pinMode(AB1IN_RIGHT,OUTPUT);
  pinMode(AB2IN_RIGHT,OUTPUT);
}
void forward() {
  Serial.println("going forward");
  analogWrite(AB1IN_LEFT,60); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,SCALEFACTOR*60); 
  analogWrite(AB2IN_RIGHT,0);
}
void backward() {
  Serial.println("going backward");
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,60);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,SCALEFACTOR*60);
}
void stop() {
  Serial.println("stopped");
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,0); 
}

void threeSixty() {
  Serial.println("360");
  analogWrite(AB1IN_LEFT,60); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,SCALEFACTOR*60);

}
void loop() {
  forward();
  delay(2000);
  stop();
  delay(2000);
  backward();
  delay(2000);
  stop();
  delay(2000);
  threeSixty();
  delay(2000);
  stop();
  delay(2000);
}

```
{% end %}

This resulted in the following motor output: 

<iframe width="600" height = "400" src="https://youtube.com/embed/iCxTIqge7iY"allowfullscreen></iframe>



With battery leads soldered and batteries attached, I performed the following test without any motor compensation:


<iframe width="600" height = "400" src="https://youtube.com/embed/ubOYevCqdug"allowfullscreen></iframe>


This was my initial setup:

<img src="/files/lab/wired.jpg" alt="wired"  width = 600 >



I found that my motors failed to drive the car forward and backward (with a fully-charged battery) up to a PWM value of around 30. It failed to turn about its own axis up to PWM 90, however, due to the increased slip friction of the wheels. All tests were performed on vinyl tiling. 

To calibrate the motor, I simply multiplied the input to the side with greater friction by a constant. It took a bit of tuning, but I landed on a constant coefficient of around 1.4 for the left motor. It's worth noting, though, that the required coefficient dropped significantly as the battery lost charge -- on the last few tests of a battery, the car kept straight with a constant of around 1.2. 

<iframe width="600" height = "400" src="https://youtube.com/embed/0G7V5y0Lh2U"allowfullscreen></iframe>

I used the following code to narrow down this constant, where the coefficient is passed via BLE.  


{% note(clickable=true,hidden = true,header = "Motor Test") %}

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

## Collaborations

I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively, and referenced [Daria's](https://pages.github.coecis.cornell.edu/dak267/dak267.github.io/#contact) and [Mavis'](https://mavisfu.github.io/lab3.html) site for wiring and local test code snippets.


# Next: [Lab 5](/fast-robots/lab5)
