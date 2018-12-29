/* Samuel Carbone
    AERO2711 Semester 2 2018
    University of Sydney
*/

/* 
    A basic PID controller for use in aircraft control
*/

#include "PID.hpp"

// Constructor sets the coefficients
PID::PID(double Kp_in, double Ki_in, double Kd_in) : Kp(Kp_in), Ki(Ki_in), Kd(Kd_in) {

    // Set the initial values of error_prev, I, D
    error_prev = 0;
    I = 0;
	D = 0;
}

// Destructor
PID::~PID() {}

// Calculate the command from the PID controller
double PID::Calculate(double error, double dt) {

    // Calculate each of the components
	CalcProp(error);
    CalcIntegral(error, dt);
    CalcDeriv(error, dt);

    // Multiply the components by the coefficients and sum
    // The result is negated such that if the aircraft's position is positive, the command is in the negative direction
    // E.g. if the target X is +1m, and the position is +2m, then Kp*P is positive, so it needs to be negated
    result = -(Kp * P + Ki * I + Kd * D);

    return result;
}

// Calculate the Integral
void PID::CalcIntegral(double error, double dt) {

    // Simple rectangle method is being used for estimating the integral
    I += dt * error;
}

// Calculate the derivative
void PID::CalcDeriv(double error, double dt) {

    // Two point backward difference approximation
    D = (error -error_prev)/dt;

    error_prev = error;
}

// Calculate the proportional term
void PID::CalcProp(double error) {

    // The proportional is the error
	P = error;
}
