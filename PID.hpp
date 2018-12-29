/* Samuel Carbone
    AERO2711 Semester 2 2018
    University of Sydney
*/

/* 
    A basic PID controller for use in aircraft control
*/

#ifndef PID_H
#define PID_H

class PID {

    public:
        PID(double Kp_in, double Ki_in, double Kd_in); // Constructor sets the coefficients
        ~PID(); // Destructor
        double PID::Calculate(double error, double dt); // Calculate the command from the PID controller

        double error_prev; // Previous value of the error, used in deriv calc
    
        // Coefficients
        double Kp;
        double Ki;
        double Kd;
    
        // Terms prior to multiplication by coefficients
		double P;
		double I;
		double D;
    
        // Final result a.k.a command
		double result;

    private:
        void PID::CalcIntegral(double error, double dt); // Calculate the Integral
        void PID::CalcDeriv(double error, double dt); // Calculate the derivative
		void PID::CalcProp(double error); // Calculate the proportional term

};

#endif
