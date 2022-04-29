#include "usv_model/model_library.h"

namespace ModelLibrary{

Viknes830::Viknes830(){
    //default_T = 100.0;
    //default_dt = 0.1;

    tau_rb = Eigen::Vector3d::Zero();
    L_  = 10;       // [m]
    W_  = 4;        // [m]
    M   = 3980;     // [kg]
    I_z = 19703.0;   // [kg/m2]

    // M_A
    X_udot = 0.0;
    Y_vdot = 0.0;
    Y_rdot = 0.0;
    N_vdot = 0.0;
    N_rdot = 0.0;

      // Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
    X_u	= -50.0;
    Y_v = -200.0;
    Y_r = 0.0;
    N_v = 0.0;
    N_r = -3224.0;

    // Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
    X_uu = -135.0;
    Y_vv = -2000.0;
    N_rr = 0.0;
    X_uuu = 0.0;
    Y_vvv = 0.0;
    N_rrr = -3224.0;

    Eigen::Matrix3d Mtot;
    Mtot << M - X_udot, 0, 0,
            0, M-Y_vdot, -Y_rdot,
            0, -Y_rdot, I_z-N_rdot;
    Minv = Mtot.inverse();

    //Force limits
    Fx_min = -6550.0;
    Fx_max = 13100.0;
    Fy_min = -645.0;
    Fy_max = 645.0;

    // Other
    rudder_d = 4.0; // distance from rudder to CG

      //Low-level controllers 
    Kp_u = 1.0;
    Kp_psi = 5.0;
    Kd_psi = 1.0;
    Kp_r = 8.0;
}

void Viknes830::operator()(const state_type& state, state_type & state_dt, const double /*t*/){
    double x = state[0];
    double y = state[1];
    double psi = SSA(state[2]);

    psi_d = normalize_angle_diff(psi_d, psi);

    double u = state[3];
    double v = state[4];
    double r = state[5];

    double r11 = cos(psi);
    double r12 = -sin(psi);
    double r21 = sin(psi);
    double r22 = cos(psi);

    // Calculate coriolis and dampening matrices according to Fossen, 2011 or Stenersen, 2014.
    Cvv(0) = (-M*v + Y_vdot*v + Y_rdot*r) * r;
    Cvv(1) = ( M*u - X_udot*u) * r;
    Cvv(2) = ((M*v - Y_vdot*v - Y_rdot*r ) * u + ( -M*u + X_udot*u) * v);

    Dvv(0) = - (X_u + X_uu*fabs(u) + X_uuu*pow(u,2)) * u;
    Dvv(1) = - ((Y_v*v + Y_r*r) + (Y_vv*fabs(v)*v + Y_vvv*pow(v,3)));
    Dvv(2) = - ((N_v*v + N_r*r) + (N_rr*fabs(r)*r + N_rrr*pow(r,3)));

    double Fx = Cvv[0] + Dvv[0] + Kp_u*M*(u_d - u);
    double Fy = ((Kp_psi * I_z ) * ((psi_d - psi) - Kd_psi*r))/rudder_d;

    // Saturate
	  if (Fx < Fx_min)
	    Fx = Fx_min;
	  if (Fx > Fx_max)
	    Fx = Fx_max;

	  if (Fy < Fy_min)
	    Fy = Fy_min;
	  if (Fy > Fy_max)
	    Fy = Fy_max;
    double Fn = rudder_d * Fy;

    tau_rb(0) = Fx;
    tau_rb(1) = Fy;
    tau_rb(2) = Fn;

    Eigen::Vector3d temp = Minv * (tau_rb - Cvv - Dvv);

    state_dt[0] = r11 * u + r12 * v;    //xdot
    state_dt[1] = r21 * u + r22 * v;    //ydot
    state_dt[2] = r;                    //psidot
    state_dt[3] = temp(0);              //xdotdot
    state_dt[4] = temp(1);              //ydotdot
    state_dt[5] = temp(2);              //psidotdot

}

simulatedHorizon Viknes830::simulateHorizon(state_type x_init, double u_d, double psi_d, double T){
    simulatedHorizon sim_hor;
    this->u_d = u_d;
    this->psi_d = psi_d;
    runge_kutta4< state_type > stepper;
    sim_hor.steps = integrate_const(stepper,*this,x_init,0.0,T,0.1,simulatedHorizonObserver(sim_hor));
    return sim_hor;
}

simulatedHorizon Viknes830::simulateHorizonAdaptive(state_type& x_init, double u_d, double psi_d, double T){
    simulatedHorizon sim_hor;
    this->u_d = u_d;
    this->psi_d = psi_d;
    sim_hor.steps = integrate(*this,x_init,0.0,T,0.1,simulatedHorizonObserver(sim_hor));
    return sim_hor;
}


void Viknes830::simulate(state_type& x, double u_d, double psi_d, double T){
    this->u_d = u_d;
    this->psi_d = psi_d;
    size_t steps = integrate(*this,x,0.0,T,0.01);
}

double SSA(double angle){
        return fmod(angle+M_PI,2*M_PI) - M_PI;
    }

double normalize_angle_diff(double angle, double angle_ref){
    double new_angle;
    double diff = angle_ref - angle;

    if (isinf(angle) || isinf(angle_ref)) return angle;

    // Get angle within 2*PI of angle_ref
    if (diff > 0){
        new_angle = angle +(diff - fmod(diff, 2*M_PI));
    }else{
        new_angle = angle + (diff + fmod(-diff, 2*M_PI));
    }

    // Get angle on side closest to angle_ref
    diff = angle_ref - new_angle;
    if (diff > M_PI){
        new_angle += 2*M_PI;
    }else if (diff < -M_PI){
        new_angle -= 2*M_PI;
    }
    return new_angle;
}

LinearObstacleShip::LinearObstacleShip(double length, double width){
  length_ = length;
  width_ = width;
}

void LinearObstacleShip::operator()(const state_type& x, state_type &dxdt, const double /*t*/){
  double u = x[3];
  double v = x[4];
  double r = x[5];

  double r11 = cos(x[2]);
  double r12 = -sin(x[2]);
  double r21 = sin(x[2]);
  double r22 = cos(x[2]);

  dxdt[0] = r11*u + r12*v;
  dxdt[1] = r21*u + r22*v;
  dxdt[2] = 0;
  dxdt[3] = 0;
  dxdt[4] = 0;
  dxdt[5] = 0;
}

simulatedHorizon LinearObstacleShip::simulateHorizon(state_type x_init, double T){
  simulatedHorizon sim_hor;
  runge_kutta4< state_type > stepper;
  sim_hor.steps = integrate_const(stepper,*this,x_init,0.0,T,0.1,simulatedHorizonObserver(sim_hor));
  return sim_hor;
}

void LinearObstacleShip::simulateToTime(state_type& x, double T){
  size_t steps = integrate(*this,x,0.0,T,0.1);
}




} // End namespace ModelLibrary