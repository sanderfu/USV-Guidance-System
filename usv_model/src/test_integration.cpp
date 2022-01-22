#include <boost/numeric/odeint.hpp>
#include "usv_model/model_library.h"

using namespace std;
using namespace boost::numeric::odeint;
#include <iostream>
#include <fstream>
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;

const double gam = 0.15;

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

struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
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
            push_back_state_and_time( x_vec , times ) );

    /* output */
    ofstream outfile;
    outfile.open("test.txt");
    for( size_t i=0; i<=steps; i++ )
    {
        cout << times[i] << '\t\t' << x_vec[i][0] << '\t\t' << x_vec[i][1] << '\n';
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
    vector<state_type> x_vec_viknes;
    vector<double> times_viknes;

    steps = integrate( viknes ,
            x_viknes , 0.0 , 100.0 , 0.1 ,
            push_back_state_and_time( x_vec_viknes , times_viknes ) );
    
    /* output */
    outfile.open("viknes_test.txt");
    for( size_t i=0; i<=steps; i++ )
    {
        //cout << times[i] << '\t\t' << x_vec[i][0] << '\t\t' << x_vec[i][1] << '\n';
        outfile << times_viknes[i] << ',' << x_vec_viknes[i][0] << ',' << x_vec_viknes[i][1] << ',' << x_vec_viknes[i][2] << ',' << x_vec_viknes[i][3] << ',' << x_vec_viknes[i][4] << ',' << x_vec_viknes[i][5] << '\n';
    }
    outfile.close();
}