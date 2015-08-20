/** 
* @author Ugo Pattacini <ugo.pattacini@iit.it>
*  
*******************************************************************************
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*******************************************************************************
*/

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <fstream>
#include <string>
#include <stdio.h>
#include <deque>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

struct Item
{
    double t;
    Vector xd;
};


inline string printVector(const Vector &v)
{
    string ret;
    char buf[255];

    for (int i=0; i<v.length(); i++)
    {    
        sprintf(buf,"%g ",v[i]);
        ret+=buf;
    }

    return ret;
}


int main(int argc, char *argv[])
{    
    Network yarp;

    if (!yarp.checkNetwork())
        return false;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("dataFile","data.txt");
    rf.configure("ICUB_ROOT",argc,argv);

    string stemPort=rf.check("name",Value("txTarget")).asString().c_str();
    double time=rf.check("execTime",Value(20.0)).asDouble();
    double ox=rf.check("ox",Value(-0.25)).asDouble();
    double oy=rf.check("oy",Value(-0.17)).asDouble();
    double oz=rf.check("oz",Value(0.07)).asDouble();
    double gain=rf.check("gain",Value(0.12)).asDouble();    

    deque<Item> L;
    ifstream fin(rf.findFile("dataFile").c_str());

    if (fin.is_open())
    {
        char line[255];

        for (;;)
        {
            fin.getline(&line[0],sizeof(line),'\n');

            string str(line);
            if (!str.length())
                break;

            Bottle b;
            b.fromString(str.c_str());

            Item item;
            item.t=time*b.get(0).asDouble();
            item.xd.resize(7);
            item.xd[0]=ox+gain*b.get(1).asDouble();
            item.xd[1]=oy+gain*b.get(2).asDouble();
            item.xd[2]=oz+gain*b.get(3).asDouble();
            item.xd[3]=b.get(4).asDouble();
            item.xd[4]=b.get(5).asDouble();
            item.xd[5]=b.get(6).asDouble();
            item.xd[6]=b.get(7).asDouble();

            L.push_back(item);
        }

        fin.close();
    }
    else
    {    
        fprintf(stdout,"dataFile %s not found!\n",rf.find("dataFile").asString().c_str());
        return -1;
    }

    BufferedPort<Vector> port;
    port.open(("/"+stemPort+"/xd:o").c_str());

    fprintf(stdout,"Press [enter] to start ... ");
    getchar();

    for (;;)
    {
        double t0=Time::now();

        for (unsigned int i=0; i<L.size(); i++)
        {
            Item &item=L[i];
            while ((Time::now()-t0)<item.t);
    
            port.prepare()=item.xd;
            port.write();

            double t=Time::now()-t0;
            double dt=1e3*(t-item.t);
            fprintf(stdout,"%g [s],\t%s%.1f [ms]   ***   %s\n",
                    t,dt>=0.0?"+":"-",dt,printVector(item.xd).c_str());
        }
	
		char chrs[2];
        fprintf(stdout,"Again ? [y/n] ... ");
        int ret=fscanf(stdin,"%c%c",&chrs[0],&chrs[1]);
        if (toupper(chrs[0])!='Y')
            break;
    }

    port.close();

    return 0;
}


