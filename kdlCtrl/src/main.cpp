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
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <gsl/gsl_math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <string>
#include <stdio.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class Controller : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    string robot;
    bool   simulation;
    int    period;
    int    joints;
    double Ts;
    double lambda;
    double lambda_min;
    double lambda_max;
    double lambda_dec;
    double lambda_inc;
    double sv_thres;
    double dist_old;
    double svMin;

    bool   gpmFlag;
    double K;
    double safeAreaRatio;
    Vector span;
    Vector alpha_min;
    Vector alpha_max;

    Matrix lim;
    Matrix JYarp;

    Vector xdYarp;

    Integrator *I;

    PolyDriver       *drv;
    IEncoders        *ienc;
    IVelocityControl *ivel;

    BufferedPort<Vector> xdPort;
    BufferedPort<Vector> voPort;
    BufferedPort<Vector> qoPort;
    BufferedPort<Vector> xoPort;

    KDL::Chain                       rightArmChain;
    KDL::ChainFkSolverPos_recursive *fksolver;
    KDL::ChainIkSolverVel_wdls      *iksolver;
    KDL::ChainJntToJacSolver        *jacsolver;
    KDL::JntArray                    q;
    KDL::Frame                       x;
    KDL::Frame                       xd;
    KDL::Jacobian                    J;

    int printCnt;

public:
    Controller(ResourceFinder &_rf) : 
    RateThread(10), rf(_rf)
    {
        drv=NULL;
        I=NULL;
        fksolver=NULL;
        iksolver=NULL;

        xdYarp.resize(1);
        printCnt=0;
    }

    virtual bool threadInit()
    {
        name=rf.check("name",Value("kdlCtrl")).asString().c_str();
        robot=rf.check("robot",Value("icub")).asString().c_str();
        simulation=rf.check("simulation",Value("off")).asString()=="on";
        period=rf.check("period",Value(20)).asInt();
        joints=rf.check("joints",Value(7)).asInt();
        lambda_min=rf.check("lambda_min",Value(0.001)).asDouble();
        lambda_max=rf.check("lambda_max",Value(10.0)).asDouble();
        lambda_dec=rf.check("lambda_dec",Value(0.2)).asDouble();
        lambda_inc=rf.check("lambda_inc",Value(1.1)).asDouble();
        sv_thres=rf.check("svn_thres",Value(0.000001)).asDouble();
        gpmFlag=rf.check("gpm",Value("on")).asString()=="on";
        K=rf.check("K",Value(10.0)).asDouble();
        safeAreaRatio=rf.check("safeAreaRatio",Value(0.9)).asDouble();

        lambda=lambda_max;
        Ts=0.001*period;

        if (!simulation)
        {
            Property option("(device remote_controlboard)");
            string remote="/"+robot+"/right_arm";
            string local="/"+name+"/right_arm";

            option.put("remote",remote.c_str());
            option.put("local",local.c_str());

            drv=new PolyDriver(option);
            if (!drv->isValid())
            {
                fprintf(stdout,"Device drivers not available!\n");
                delete drv;
                return false;
            }

            drv->view(ienc);
            drv->view(ivel);
        }

        Vector q0(joints);
        lim.resize(joints,2);        

        for (int i=0; i<joints; i++)
        {
            char jointName[10];
            sprintf(jointName,"joint_%d",i);
            Bottle &block=rf.findGroup(jointName);

            if (block.isNull())
                return false;

            lim(i,0)=CTRL_DEG2RAD*block.check("min",Value(0.0)).asDouble();
            lim(i,1)=CTRL_DEG2RAD*block.check("max",Value(0.0)).asDouble();
            q0[i]=CTRL_DEG2RAD*block.check("q0",Value(0.0)).asDouble();
        }

        string fwslash="/";
        xdPort.open((fwslash+name+fwslash+"xd:i").c_str());
        voPort.open((fwslash+name+fwslash+"v:o").c_str());
        qoPort.open((fwslash+name+fwslash+"q:o").c_str());
        xoPort.open((fwslash+name+fwslash+"x:o").c_str());

        setRate(period);        

        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation(0.0,-1.0,0.0, 0.0,0.0,-1.0, 1.0,0.0,0.0))));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame().DH( 0.0320000,  M_PI/2.0,  0.00000,  0.0               )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame().DH( 0.0000000,  M_PI/2.0, -0.00550, -M_PI/2.0          )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame().DH(-0.0233647,  M_PI/2.0, -0.14330, -(105.0/180.0)*M_PI)));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0000000,  M_PI/2.0, -0.10774, -M_PI/2.0          )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0000000, -M_PI/2.0,  0.00000, -M_PI/2.0          )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH(-0.0150000, -M_PI/2.0, -0.15228, -(105.0/180.0)*M_PI)));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0150000,  M_PI/2.0,  0.00000,  0.0               )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0000000,  M_PI/2.0, -0.13730, -M_PI/2.0          )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0000000,  M_PI/2.0,  0.00000,  M_PI/2.0          )));
        rightArmChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame().DH( 0.0625000,  0.000000,  0.01600,  M_PI              )));

        q.resize(joints);
        J.resize(joints);
        JYarp.resize(J.rows(),J.columns());
        span.resize(joints);
        alpha_min.resize(joints);
        alpha_max.resize(joints);

        if (!simulation)
        {
            // get initial feedback
            Vector encs(16);
            while (!ienc->getEncoders(encs.data()));
            for (int i=0; i<joints; i++)
                q0[i]=CTRL_DEG2RAD*encs[i];
        }

        I=new Integrator(Ts,q0,lim);

        for (int i=0; i<joints; i++)
            q(i)=q0[i];

        fksolver=new  KDL::ChainFkSolverPos_recursive(rightArmChain);
        iksolver=new  KDL::ChainIkSolverVel_wdls(rightArmChain);
        jacsolver=new KDL::ChainJntToJacSolver(rightArmChain);

        fksolver->JntToCart(q,x);
        iksolver->setLambda(lambda);
        xd=x;

        dist_old=1.0;
        setSafeAreaRatio();        

        return true;        
    }

    void update_lambda(KDL::Twist &err)
    {
        Matrix A;
        if (JYarp.rows()>JYarp.cols())
            A=JYarp;
        else
            A=JYarp.transposed();

        int m=A.rows();
        int n=A.cols();
        Matrix U(m,n);
        Vector Sdiag(n);
        Matrix V(n,n);
        SVD(A,U,Sdiag,V);

        svMin=Sdiag[n-1];

        if (svMin>sv_thres)
        {
            double dist=0.0;
            for (int i=0; i<6; i++)
                dist+=err(i)*err(i);

            dist=sqrt(dist);

            double ratio=dist/dist_old;

            if (ratio>1.0)
                lambda*=lambda_inc;
            else
                lambda*=lambda_dec;

            lambda=lambda>lambda_max?lambda_max:(lambda<lambda_min?lambda_min:lambda); 
            dist_old=dist;
        }
        else
        {
            lambda=lambda_max;
            dist_old=1.0;
        }

        iksolver->setLambda(lambda);        
    }

    void setSafeAreaRatio()
    {
        for (int i=0; i<joints; i++)
        {
            span[i]=lim(i,1)-lim(i,0);

            double alpha=0.5*(1.0-safeAreaRatio)*span[i];
            alpha_min[i]=lim(i,0)+alpha;
            alpha_max[i]=lim(i,1)-alpha;
        }
    }

    Vector computeGPM()
    {        
        if (gpmFlag)
        {
            Vector w(joints);
            Vector qYarp(joints);

            for (int i=0; i<joints; i++)
                qYarp[i]=q(i);

            Vector d_min=qYarp-alpha_min;
            Vector d_max=qYarp-alpha_max;

            for (int i=0; i<joints; i++)
            {
                w[i] =d_min(i)>0.0 ? 0.0 : 2.0*d_min(i)/(span[i]*span[i]);
                w[i]+=d_max(i)<0.0 ? 0.0 : 2.0*d_max(i)/(span[i]*span[i]);
            }

            Matrix JYarpT=JYarp.transposed();
            Matrix LM=JYarp*JYarpT+lambda*lambda*eye(6,6);
            Matrix pinvLM;
            if (LM.rows()>LM.cols())
                pinvLM=JYarpT*pinv(LM);
            else
                pinvLM=JYarpT*pinv(LM.transposed()).transposed();

            return(eye(joints,joints)-pinvLM*JYarp)*((-K)*w);
        }
        else
        {
            Vector gpm(joints);
            gpm=0.0;

            return gpm;
        }
    }

    Vector checkVelocity(const Vector &_qdot, const KDL::JntArray &_q)
    {
        Vector checked_qdot=_qdot;

        for (int i=0; i<joints; i++)
        {
            double newqi=_q(i)+Ts*checked_qdot[i];
            if (newqi<lim(i,0) || newqi>lim(i,1))
                checked_qdot[i]=0.0;
        }

        return checked_qdot;
    }

    virtual void run()
    {
        KDL::Twist    err;
        KDL::JntArray qdot(joints);
        Matrix R(3,3);

        if (!simulation)
        {
            Vector encs(16);
            if (ienc->getEncoders(encs.data()))
                for (int i=0; i<joints; i++)
                    q(i)=CTRL_DEG2RAD*encs[i];
        }

        fksolver->JntToCart(q,x);
        jacsolver->JntToJac(q,J);

        for (int i=0; i<JYarp.rows(); i++)
            for (int j=0; j<JYarp.cols(); j++)
                JYarp(i,j)=J(i,j);

        if (Vector *xdNew=xdPort.read(false))
        {
            xdYarp=*xdNew;

            Vector axis(4);
            axis[0]=xdYarp[3];
            axis[1]=xdYarp[4];
            axis[2]=xdYarp[5];
            axis[3]=xdYarp[6];
            Matrix H=axis2dcm(axis);

            xd=KDL::Frame(KDL::Rotation(H(0,0),H(0,1),H(0,2), H(1,0),H(1,1),H(1,2), H(2,0),H(2,1),H(2,2)),
                          KDL::Vector(xdYarp[0],xdYarp[1],xdYarp[2]));
        }

        KDL::Frame e=xd*x.Inverse();
        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                R(i,j)=e.M(i,j);

        Vector axis=dcm2axis(R);

        err(0)=xd.p(0)-x.p(0);
        err(1)=xd.p(1)-x.p(1);
        err(2)=xd.p(2)-x.p(2);
        err(3)=axis[3]*axis[0];
        err(4)=axis[3]*axis[1];
        err(5)=axis[3]*axis[2];

        iksolver->CartToJnt(q,err,qdot);
        Vector gpm=computeGPM();

        update_lambda(err);

        Vector qdotFinal(joints);
        for (int i=0; i<joints; i++)
            qdotFinal[i]=qdot(i)+gpm[i];

        qdotFinal=checkVelocity(qdotFinal,q);

        Vector qYarp=I->integrate(qdotFinal);
        for (int i=0; i<joints; i++)
            q(i)=qYarp[i];

        Vector &qo=qoPort.prepare();
        Vector &xo=xoPort.prepare();
        Vector &vo=voPort.prepare();

        qo.resize(3+joints,0.0);
        vo.resize(3+joints,0.0);

        for (int i=0; i<joints; i++)
        {
            qo[3+i]=CTRL_RAD2DEG*qYarp[i];
            vo[3+i]=CTRL_RAD2DEG*qdotFinal[i];
        }

        if (!simulation)
        {
            Vector v(16); v=0.0;
            for (int i=0; i<joints; i++)
                v[i]=CTRL_RAD2DEG*qdotFinal[i];

            ivel->velocityMove(v.data());
        }

        for (int i=0; i<3; i++)
            for (int j=0; j<3; j++)
                R(i,j)=x.M(i,j);

        axis=dcm2axis(R);

        xo.resize(7);
        xo[0]=x.p(0);
        xo[1]=x.p(1);
        xo[2]=x.p(2);
        xo[3]=axis[0];
        xo[4]=axis[1];
        xo[5]=axis[2];
        xo[6]=axis[3];

        if (++printCnt>10 && xdYarp.length()==7)
        {
            fprintf(stdout,"********************\n");
            fprintf(stdout,"xd=%s\n",xdYarp.toString().c_str());
            fprintf(stdout,"x =%s\n",xo.toString().c_str());
            fprintf(stdout,"min sv=%g\n",svMin);
            fprintf(stdout,"lambda=%g\n",lambda);
            fprintf(stdout,"gpm=%s\n",(CTRL_RAD2DEG*gpm).toString().c_str());
            fprintf(stdout,"\n");

            printCnt=0;
        }

        qoPort.write();
        xoPort.write();
        voPort.write();
    }

    virtual void threadRelease()
    {
        if (drv)
            delete drv;

        if (I)
            delete I;

        if (fksolver)
            delete fksolver;

        if (iksolver)
            delete iksolver;

        if (jacsolver)
            delete jacsolver;

        xdPort.interrupt();
        voPort.interrupt();
        qoPort.interrupt();
        xoPort.interrupt();

        xdPort.close();
        voPort.close();
        qoPort.close();
        xoPort.close();
    }
};


class module: public RFModule
{
protected:
    Controller *ctrl;
    Port        rpcPort;

public:
    module() {}

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        ctrl=new Controller(rf);
        if (!ctrl->start())
        {
            delete ctrl;
            return false;
        }

        string name=rf.check("name",Value("kdlCtrl")).asString().c_str();
        string rpcPortName="/"+name+"/rpc";

        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        ctrl->stop();
        delete ctrl;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
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


