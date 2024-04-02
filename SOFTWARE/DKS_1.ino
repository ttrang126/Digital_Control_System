#include <math.h>
#define pi 3.14159265358979323846

// Inverter parameter
float freq_sth = 10000.00;                  // Switching freqency 5000Hz
float freq = 50.0;                          // Variable fundamental freqency 50
float sample_time = pow(10,-5);             // sample_time(per cycle) = switching_time/fundamental_output_freq.
float vdc = 24;                             // rectified voltage                            
float m;                                    // modulation index (Controls max Vref magnitude) 0>=m<=1
float Vref = (vdc)*(m);
float Ua_ref = 12*sin(2*pi*freq);
float Ub_ref = 12*sin(2*pi*freq - (2/3)*pi);
float Uc_ref = 12*sin(2*pi*freq + (2/3)*pi);

// SVPWM 
float tz = ((1/freq_sth))/pow(10,-6);        // Total switching time Tz in (us)
float timer1 = 0, timer2 = 0, timer0 = 0;    // Switching times
int sector_pos = 1;
float timer_div = 0;                         // Constant in time calculation equations. (refer to t1,t2 equations)
double deta = 0, theta = 0;                  // trignometric part of t1,t2 equations, computated seperately to reduce clutering of code in one line.(i.e t1 = timer_div*theta1)

// Sensor value
float Ia = 0, Ib = 0, Ic = 0, Id = 0, Iq = 0;
float ILa = 0, ILb = 0, ILc = 0, ILd = 0, ILq = 0;
float Ua = 0, Ub = 0, Uc = 0, Ud = 0, Uq = 0;
double U_alpha = 0, U_beta = 0, Vr = 0;

int ig1 = 6, ig3 = 5, ig5 = 3, ig2 = 0, ig4 = 0, ig6 = 0;

//Specify the links and initial tuning parameters
double v_Kp = 0.09, v_Ki= 20.8;
double i_Kp = 16.5, i_Ki = 33;
float deltaT = 1, wC = 0.0147, wL = 1.0367;
float eUdintegral = 0, eUqintegral = 0, eUd = 0, eUq = 0;
float eIdintegral = 0, eIqintegral = 0, eId = 0, eIq = 0;
float Ud_sp = 0, Uq_sp = 0, Idref = 0, Iqref = 0, Udref = 0, Uqref = 0, Isd = 0, Isq = 0, Usq = 0, Usd = 0;

void setup() {
// put your setup code here, to run once:
Serial.begin(9600);

pinMode(3, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);

pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
pinMode(A4, INPUT);
pinMode(A5, INPUT);
pinMode(A9, INPUT);
pinMode(A10, INPUT);
pinMode(A11, INPUT);
}
void loop() {
// put your main code here, to run repeatedly:

// Read data from Sensor
Ia = analogRead(A3);
Ib = analogRead(A4);
Ic = analogRead(A5);

ILa = analogRead(A9);
ILb = analogRead(A10);
ILc = analogRead(A11);

Ua = analogRead(A2);
Ub = analogRead(A1);
Uc = analogRead(A0);

// park va clark transform abc -> dq
// controller
// inverse park dq -> alpha beta
// svpwm
controller();
sector_checker();
time_calculate();
mosfet_switch();
}

void controller(){
theta = 2*pi*freq*sample_time;
// Park transformation of Uabc from abc to dq frame
Ud_sp = 2/3*(cos(theta)*Ua_ref + cos(theta - 2*pi/3)*Ub_ref + cos(theta + 2*pi/3)*Uc_ref);
Uq_sp = 2/3*(-sin(theta)*Ua_ref - sin(theta - 2*pi/3)*Ub_ref - sin(theta + 2*pi/3)*Uc_ref); 

// Park transformation of Uabc from abc to dq frame
Ud = 2/3*(cos(theta)*Ua + cos(theta - 2*pi/3)*Ub + cos(theta + 2*pi/3)*Uc);
Uq = 2/3*(-sin(theta)*Ua - sin(theta - 2*pi/3)*Ub - sin(theta + 2*pi/3)*Uc);

// Park transformation of ULabc from abc to dq frame
ILd = 2/3*(cos(theta)*ILa + cos(theta - 2*pi/3)*ILb + cos(theta + 2*pi/3)*ILc);
ILq = 2/3*(-sin(theta)*ILa - sin(theta - 2*pi/3)*ILb - sin(theta + 2*pi/3)*ILc);

// Park transformation of Iabc from abc to dq frame
Id = 2/3*(cos(theta)*Ia + cos(theta - 2*pi/3)*Ib + cos(theta + 2*pi/3)*Ic);
Iq = 2/3*(-sin(theta)*Ia - sin(theta - 2*pi/3)*Ib - sin(theta + 2*pi/3)*Ic);

//PI Controller
//Current control loop
eUd = Ud_sp - Ud;
eUq = Uq_sp - Uq;
eUdintegral = eUdintegral + eUd*sample_time;
eUqintegral = eUqintegral + eUq*sample_time;
Idref = v_Kp*eUd + v_Ki*eUdintegral;
Iqref = v_Kp*eUq + v_Ki*eUqintegral;

Isd = Idref + ILd - wC*Ud;
Isq = Iqref + ILq + wC*Uq;

//Volatge control loop
eId= Isd - Id;
eUq= Isq - Iq;
eIdintegral = eIdintegral + eId*sample_time;
eIqintegral = eIqintegral + eIq*sample_time;
Udref= i_Kp*eId + i_Ki*eIdintegral;
Uqref= i_Kp*eIq + i_Ki*eIqintegral;

Usd= Udref - wL*Id;
Usq= Uqref + wL*Iq;

// Clark Transformation from dq to alpha beta
U_alpha = cos(theta)*Usd - sin(theta)*Usq;
U_beta = sin(theta)*Usd + cos(theta)*Usq;
}

void sector_checker()
{
if((U_beta >= 0) & (U_beta < (sqrt(3)*U_alpha)))
  sector_pos=1;
if((U_beta >= sqrt(3)*U_alpha) & (U_beta > (-sqrt(3)*U_alpha)))
  sector_pos=2;
if((U_beta >= 0) & (U_beta < (-sqrt(3)*U_alpha)))
  sector_pos=3;
if((U_beta < 0) & (U_beta >= (sqrt(3)*U_alpha)))
  sector_pos=4;
if((U_beta < (sqrt(3)*U_alpha)) & (U_beta <= (-sqrt(3)*U_alpha)))
  sector_pos=5;
if((U_beta < 0) & (U_beta >= -sqrt(3)*U_alpha))
  sector_pos=6; 
}
void time_calculate()
{
double phi = atan2(U_beta,U_alpha);
if(sector_pos == 1)
  deta = phi;
if(sector_pos == 2)
  deta = phi - pi/3;
if(sector_pos == 3)
  deta = phi - 2*pi/3;
if(sector_pos == 4)
  deta = phi + pi;
if(sector_pos == 5)
  deta = phi + 2*pi/3;
if(sector_pos == 6)
  deta = phi + pi/3; 

Vref = sqrt((U_alpha*U_alpha) + (U_beta*U_beta));
m = Vref/(2/3*vdc);

// Time share
timer1 = tz*m*sin((pi/3) - deta)/sin(pi/3);
timer2 = tz*m*sin(deta)/sin(pi/3);
timer0 = tz*m - timer1 - timer2;
// all time in us
timer1 = round(timer1);
timer2 = round(timer2);
timer0 = round(timer0); 

}
void mosfet_switch() {
//Symmetric switching pattern/sequences used for optimal results.(low THD etc)
//refer to 'document' for rules for each V-state switches. 
//switches in each V-state are defined by table( using the one given at wikipedia)
//v0-v1-v2-v7-v2-v1-v0
//-----
//v0:
if (sector_pos == 1){
  digitalWrite(ig1,LOW); 
  digitalWrite(ig3,LOW); 
  digitalWrite(ig5,LOW); 

  delayMicroseconds(timer0);
//v1:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);
//v2:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v2:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v1:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
}
//-------------------------------
if (sector_pos == 2)
{
//v0-v3-v2-v7-v2-v3-v0
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
//v3:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v2:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);

//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v2:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);
//v3:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);


  delayMicroseconds(timer0);
}
//-------------------------------
if (sector_pos == 3)
{
//v0-v3-v4-v7-v4-v3-v0
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
//v3:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);
//v4:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v4:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v3:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer1);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
}
//-------------------------------
if (sector_pos == 4)
{
//v0-v5-v4-v7-v4-v5-v0
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
//v5:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v4:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v4:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v5:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
}
//-------------------------------
if (sector_pos == 5)
{
//v0-v5-v6-v7-v6-v5-v0
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
//v5:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v6:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v6:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer2);
//v5:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
}
//-------------------------------
if (sector_pos == 6)
{
//v0-v1-v6-v7-v6-v1-v0
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0);
//v1:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v6:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v7:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,HIGH);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer0);
//v6:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,HIGH);

  delayMicroseconds(timer1);
//v1:
  digitalWrite(ig1,HIGH);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer2);
//v0:
  digitalWrite(ig1,LOW);
  digitalWrite(ig3,LOW);
  digitalWrite(ig5,LOW);

  delayMicroseconds(timer0); 
}

}
