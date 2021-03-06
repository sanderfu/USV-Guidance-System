#pragma once
#include <eigen3/Eigen/Dense>
#include <gdal/ogrsf_frmts.h>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;
typedef std::vector< double > state_type;

namespace ModelLibrary{

    struct simulatedHorizon{
        std::vector< state_type > state;
        std::vector< double > time;
        size_t steps;

        /*
        void operator()(const simulatedHorizon& sim_hor){
            state = sim_hor.state;
            time = sim_hor.time;
        }
        */
    };

    struct simulatedHorizonObserver
    {
        simulatedHorizon& sim_hor_;

        simulatedHorizonObserver(simulatedHorizon& sim_hor)
        :  sim_hor_(sim_hor){ }

        void operator()( const state_type &x , double t )
        {
            sim_hor_.state.push_back( x );
            sim_hor_.time.push_back( t );
        }
    };

    class Viknes830{
        public:
            Viknes830();

            // ODE functions dxdt = f(x) must be defined in this operator for odeint to work
            void operator() (const state_type& x, state_type &dxdt, const double /*t*/);

            simulatedHorizon simulateHorizon(state_type x_init, double u_d, double psi_d, double T);
            simulatedHorizon simulateHorizonAdaptive(state_type& x_init, double u_d, double psi_d, double T);
            void simulate(state_type& x, double u_d, double psi_d, double T);

            inline double getL() {return L_;}
            inline double getW() {return W_;}
        private:
            state_type pose; //x,y,psi
            state_type twist; //u,v,r

            static double default_T;
            static double default_dt;

            double u_d=5;
            double psi_d=M_PI/4;

            Eigen::Matrix3d Minv;
            Eigen::Vector3d Cvv; // C(v)v
            Eigen::Vector3d Dvv; // D(v)v
            Eigen::Vector3d tau_rb;


            // Model Parameters
            double M; 	// [kg]
            double I_z; // [kg/m2]
            double L_, W_;
            
            // Added mass terms
            double X_udot;
            double Y_vdot;
            double Y_rdot;
            double N_vdot;
            double N_rdot;

            // Linear damping terms [X_u, Y_v, Y_r, N_v, N_r]
            double X_u;
            double Y_v;
            double Y_r;
            double N_v;
            double N_r;

            // Nonlinear damping terms [X_|u|u, Y_|v|v, N_|r|r, X_uuu, Y_vvv, N_rrr]
            double X_uu;
            double Y_vv;
            double N_rr;
            double X_uuu;
            double Y_vvv;
            double N_rrr;

            //Force limits
            double Fx_min;
            double Fx_max;
            double Fy_min;
            double Fy_max;

            // Other
            double rudder_d;

            //Low-level speed and course controller
            double Kp_u;
            double Kp_psi;
            double Kd_psi;
            double Kp_r;
    };

    class LinearObstacleShip{
        public:
            LinearObstacleShip(double length, double width);
            void operator()(const state_type& x, state_type &dxdt, const double /*t*/);
            simulatedHorizon simulateHorizon(state_type x_init, double T);
            void simulateToTime(state_type& x_init, double T);
            inline double getL() {return length_;}
            inline double getW() {return width_;}
        private:
            double length_;
            double width_;
            state_type x_init_;


    };


    double SSA(double angle);
    double normalize_angle_diff(double angle, double angle_ref);


}//End namespace ModelLibrary;