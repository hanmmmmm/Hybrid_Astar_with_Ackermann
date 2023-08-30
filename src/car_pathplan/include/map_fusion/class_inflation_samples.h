#ifndef CLASS_INFLATION_SAMPLES
#define CLASS_INFLATION_SAMPLES

#include <vector>
#include <iostream>

class ClassInflationSamples
{
private:
    int8_t obs_0;
    int8_t clr__;
    int8_t obs_1;
    int8_t obs_2;
    int8_t obs_3;
public:
    ClassInflationSamples();
    ~ClassInflationSamples();

    void get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius);
};

ClassInflationSamples::ClassInflationSamples()
{
    obs_0 = 100;
    clr__ = 0; 
    obs_1 = 90;
    obs_2 = 80;
    obs_3 = 70;
}

ClassInflationSamples::~ClassInflationSamples()
{
}

void ClassInflationSamples::get_sample_by_radius(std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius){
    if( radius == 1){
        inflate_sample_target.push_back( {obs_2, obs_1, obs_2} );
        inflate_sample_target.push_back( {obs_1, obs_0, obs_1} );
        inflate_sample_target.push_back( {obs_2, obs_1, obs_2} );
    }
    else if( radius == 2){
        inflate_sample_target.push_back( {clr__, obs_3, obs_2, obs_3, clr__} );
        inflate_sample_target.push_back( {obs_3, obs_1, obs_1, obs_1, obs_3} );
        inflate_sample_target.push_back( {obs_2, obs_1, obs_0, obs_1, obs_2} );
        inflate_sample_target.push_back( {obs_3, obs_1, obs_1, obs_1, obs_3} );
        inflate_sample_target.push_back( {clr__, obs_3, obs_2, obs_3, clr__} );
    }
    else if( radius == 3){
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_0,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
    }
    else if( radius >= 4){
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_0,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {obs_3,obs_2,obs_1,obs_1,obs_1,obs_2,obs_3} );
        inflate_sample_target.push_back( {clr__,obs_2,obs_2,obs_2,obs_2,obs_2,clr__} );
        inflate_sample_target.push_back( {clr__,clr__,obs_3,obs_3,obs_3,clr__,clr__} );
    }
}







#endif