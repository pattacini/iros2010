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
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/pids.h>

#include <string>
#include <stdio.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class velocityController : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    int    period;
    int    joints;

    Vector Kp;
    Vector maxVel;
    Vector qd;
    Vector q;

    Matrix lim;

    Integrator *I;    

    BufferedPort<Vector> qdPort;
    BufferedPort<Vector> voPort;
    BufferedPort<Vector> qoPort;

public:
    velocityController(ResourceFinder &_rf) : 
                       RateThread(20), rf(_rf)
    {
        I=NULL;
    }

    virtual bool threadInit()
    {
        name=rf.check("name",Value("simVelCtrl")).asString().c_str();
        period=rf.check("period",Value(20)).asInt();
        joints=rf.check("joints",Value(7)).asInt();

        Kp.resize(joints);
        maxVel.resize(joints);
        q.resize(joints);
        lim.resize(joints,2);        

        for (int i=0; i<joints; i++)
        {
            char jointName[10];
            sprintf(jointName,"joint_%d",i);
            Bottle &block=rf.findGroup(jointName);

            if (block.isNull())
                return false;

            Kp[i]=block.check("Kp",Value(0.0)).asDouble();
            maxVel[i]=block.check("max_vel",Value(0.0)).asDouble();
            lim(i,0)=block.check("min",Value(0.0)).asDouble();
            lim(i,1)=block.check("max",Value(0.0)).asDouble();
            q[i]=block.check("q0",Value(0.0)).asDouble();
        }

        string fwslash="/";
        qdPort.open((fwslash+name+fwslash+"qd:i").c_str());
        voPort.open((fwslash+name+fwslash+"v:o").c_str());
        qoPort.open((fwslash+name+fwslash+"q:o").c_str());

        setRate(period);

        I=new Integrator(period*0.001,q,lim);

        qd=q;

        return true;
    }

    virtual void run()
    {
        if (Vector *qdNew=qdPort.read(false))
            qd=*qdNew;

        Vector v=Kp*(qd-q);

        for (int i=0; i<joints; i++)
            if (v[i]>maxVel[i])
                v[i]=maxVel[i];
            else if (v[i]<-maxVel[i])
                v[i]=-maxVel[i];

        q=I->integrate(v);

        Vector &qo=qoPort.prepare();
        Vector &vo=voPort.prepare();

        qo.resize(3+joints,0.0);
        vo.resize(3+joints,0.0);

        // deal with torso joints
        for (int i=3; i<qo.length(); i++)
        {
            qo[i]=q[i-3];
            vo[i]=v[i-3];
        }

        qoPort.write();
        voPort.write();
    }

    virtual void threadRelease()
    {
        if (I)
            delete I;

        qdPort.interrupt();
        voPort.interrupt();
        qoPort.interrupt();

        qdPort.close();
        voPort.close();
        qoPort.close();
    }
};


class module: public RFModule
{
protected:
    velocityController *velCtrl;

public:
    module() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        velCtrl=new velocityController(rf);
        if (!velCtrl->start())
        {
            delete velCtrl;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        velCtrl->stop();
        delete velCtrl;

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        if (velCtrl->getIterations()>1000)
        {
            fprintf(stdout,"*** statistics reset\n");    
            velCtrl->resetStat();
        }

        fprintf(stdout,"--- period [s] = %g, run [s] = %g\n",
                velCtrl->getEstPeriod()/1000.0,velCtrl->getEstUsed()/1000.0);

        return true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    module mod;

    return mod.runModule(rf);
}


