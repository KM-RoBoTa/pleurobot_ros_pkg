#include "controller.hpp"

#include <optimization/webots.hh>
#include <sstream>



#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Servo.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#define nParameters 5
#define nSettings 1



#ifdef OPTIMIZATION
using namespace optimization::messages::task;
using namespace std;




void
Controller :: optimizationInit(){
    // get pointer to the webots opti stuff
    optimization::Webots &opti = optimization::Webots::Instance();
    if(opti){
        // get names of parameters and settings
        for(int i=0; i<nParameters; i++)
        {
        string str="P";
            stringstream s;
                    s << "P";
                    s << i; cout<<s.str()<<endl;
            params_names.push_back(string(s.str().c_str()));
        }
        for(int i=0; i<nSettings; i++)
        {
            stringstream s("");
                    s << "S";
                    s << i;
            settings_names.push_back(s.str());
        }

        // get parameters
        Task::Parameter p;
        for(int i=0; i<params_names.size(); i++)
        {
            if (opti.Parameter(params_names[i],p))
                {
                params.push_back(p.value());
            }
            else
            {
                cerr << "optimization parameter ' " << params_names[i] << " ' not found! " << endl;
            }
        }

        // get settings
        string param;
        double val;

        for(int i=0; i<settings_names.size(); i++)
        {
            if (opti.Setting(settings_names[i], param))
                {
                    stringstream s(param);
                    s >> val;
                settings.push_back(val); cout<<settings[i]<<endl;
            }
            else
            {
                cerr << "optimization setting ' " << settings_names[i] << " ' not found! " << endl;
            }
        }

        /////////////////////////////////// modify stuff ////////////////////////////////////////////
        // TAIL
        /*
        for(int i=0; i<5; i++){
            amps0(6+i)=params[i];
            phases0(6+i)=params[i+5];
        } */

        // STUMBLE REFLEX
  /*      stuRefForceLimX=params[0];
        stuRefForceLimZ=params[1];
        stuRefOnFilt=params[2];
        stuRefOffFilt=params[3];
        stuRefDx=params[4];
        stuRefDz=params[5];*/

        // EXTENSION REFLEX
        forceFilterExt=params[0];
        extRefForceLim=params[1];
        extRefOnFilter=params[2];
        extRefOffFilter=params[3];
        extRefSpike=params[4];


    }
}


void
Controller :: optimizationEnd(){
    optimization::Webots &opti = optimization::Webots::Instance();
    vector<double> fit;
    vector<string> fitness_names;
    map<string, double> fitness;

    // fitness variables - DEFINE FITHESSES HERE
    double fit1 = rolling;
    double fit2 = pitching;
    string fit1_name = "roll";
    string fit2_name = "pitch";
    fit.push_back(fit1);
    fit.push_back(fit2);
    fitness_names.push_back(fit1_name);
    fitness_names.push_back(fit2_name);





    // pack them
    for(int i=0; i<fit.size(); i++)
	{
		fitness[fitness_names[i]] = fit[i];
	}




    // Send the actual response
	if (opti.Setting("optiextractor"))
	{
		cout << "Not sending response, because in optiextractor mode! Enjoy the show!" << endl;
	}
	else
	{
		opti.Respond(fitness);
	}




    // cout if not in optimization
	if(opti.Setting("optiextractor") || !opti)
	{
		cout << "Fitness: " << fit1 << endl;
	}
/*
	// quit simulation
	if (opti && !opti.Setting("optiextractor"))
		simulationQuit(0);
*/
}



int
Controller :: optimizationShouldWeStopIt(double timestep){
    static double t_stop=0;
    t_stop+=timestep;
    optimization::Webots &opti = optimization::Webots::Instance();
    int crit=0;
    if(opti){
        if(t_stop>settings[0]){
            crit=1;
        }

    }

    return crit;
}

#endif
