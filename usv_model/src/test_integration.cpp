#include <boost/numeric/odeint.hpp>
#include "usv_model/model_library.h"

using namespace std;
using namespace boost::numeric::odeint;
#include <iostream>
#include <fstream>
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;

const double gam = 0.15;

struct state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};

/* The rhs of x' = f(x) defined as a class */
class harm_osc {

    double m_gam;

public:
    harm_osc( double gam ) : m_gam(gam) { }

    void operator() ( const state_type &x , state_type &dxdt , const double /* t */ )
    {
        dxdt[0] = x[1];
        dxdt[1] = -x[0] - m_gam*x[1];
    }
};


int main(){
    vector<state_type> x_vec;
    vector<double> times;
    harm_osc ho(0.15);

    state_type x(2);
    x[0] = 1.0; // start at x=1.0, p=0.0
    x[1] = 0.0;


    size_t steps = integrate( ho ,
            x , 0.0 , 100.0 , 0.1 ,
            state_and_time( x_vec , times ) );

    /* output */
    ofstream outfile;
    outfile.open("test.txt");
    for( size_t i=0; i<=steps; i++ )
    {
        //cout << times[i] << '\t\t' << x_vec[i][0] << '\t\t' << x_vec[i][1] << '\n';
        outfile << times[i] << ',' << x_vec[i][0] << ',' << x_vec[i][1] << '\n';
    }
    outfile.close();

    ModelLibrary::Viknes830 viknes;
    state_type x_viknes(6);
    x_viknes[0] = 0;
    x_viknes[1] = 0;
    x_viknes[2] = 0;
    x_viknes[3] = 0;
    x_viknes[4] = 0;
    x_viknes[5] = 0;
    ModelLibrary::simulatedHorizon sim_hor = viknes.simulateHorizon(x_viknes,5,M_PI/4,100);
    viknes.simulate(x_viknes,5,M_PI/4,100);

    /*
    steps = integrate( viknes ,
            x_viknes , 0.0 , 100.0 , 0.1 ,
            state_and_time( x_vec_viknes , times_viknes ) );
    */

    /* output */
    outfile.open("viknes_test.txt");
    for( size_t i=0; i<=sim_hor.steps; i++ )
    {
        //cout << times[i] << '\t\t' << x_vec[i][0] << '\t\t' << x_vec[i][1] << '\n';
        cout << sim_hor.time[i] << ',' << sim_hor.state[i][0] << ',' << sim_hor.state[i][1] << ',' << sim_hor.state[i][2] << ',' << sim_hor.state[i][3] << ',' << sim_hor.state[i][4] << ',' << sim_hor.state[i][5] << '\n';
    }
    outfile.close();
    cout << x_viknes[0] << " " << x_viknes[1] << " " << x_viknes[2]<< " " << x_viknes[3] << " " << x_viknes[4] << " " << x_viknes[5] << std::endl;

    state_type x_lin_obst(6);
    x_lin_obst[0] = 0;
    x_lin_obst[1] = 0;
    x_lin_obst[2] = M_PI/4;
    x_lin_obst[3] = 5;
    x_lin_obst[4] = 0;
    x_lin_obst[5] = 0;
    ModelLibrary::LinearObstacleShip obst(x_lin_obst,5,2);
    ModelLibrary::simulatedHorizon sim_hor_obst = obst.simulate(x_lin_obst, 100, 100);
     /* output */
    outfile.open("obst_test.txt");
    for( size_t i=0; i<=sim_hor_obst.steps; i++ )
    {
        //cout << times[i] << '\t\t' << x_vec[i][0] << '\t\t' << x_vec[i][1] << '\n';
        outfile << sim_hor_obst.time[i] << ',' << sim_hor_obst.state[i][0] << ',' << sim_hor_obst.state[i][1] << ',' << sim_hor_obst.state[i][2] << ',' << sim_hor_obst.state[i][3] << ',' << sim_hor_obst.state[i][4] << ',' << sim_hor_obst.state[i][5] << '\n';
    }
    outfile.close();


}