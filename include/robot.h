/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROBOT_H
#define ROBOT_H

#include "physics/pworld.h"
#include "physics/pcylinder.h"
#include "physics/pbox.h"
#include "physics/pball.h"
#include "configwidget.h"

#define ROBOT_GRAY 0.4

enum KickStatus
{
    NO_KICK = 0,
    FLAT_KICK = 1,
    CHIP_KICK = 2,
};

class CRobot
{
    PWorld *w;
    PBall *m_ball;
    dReal m_x, m_y, m_z;
    dReal m_r, m_g, m_b;
    dReal m_dir;
    int m_rob_id;
    bool firsttime;
    bool last_state{};

public:
    ConfigWidget *cfg;
    dSpaceID space;
    PObject *chassis;
    PBox *dummy;
    dJointID dummy_to_chassis;
    PBox *boxes[3]{};
    bool on;
    //these values are not controled by this class
    bool selected{};
    dReal select_x{}, select_y{}, select_z{};
    QImage *img{}, *number{};
    class Wheel
    {
    public:
        int id;
        Wheel(CRobot *robot, int _id, dReal ang, dReal ang2, int wheeltexid);
        void step();
        dJointID joint;
        dJointID motor;
        PCylinder *cyl;
        dReal speed;
        CRobot *rob;
    } * wheels[2]{};

    class RBall
    {
    public:
        int id;
        RBall(CRobot *robot, int _id, dReal ang, dReal ang2);
        void step();
        dJointID joint;
        PBall *pBall;
        dReal speed;
        CRobot *rob;
    } * balls[2]{};

    CRobot(PWorld *world, PBall *ball, ConfigWidget *_cfg, dReal x, dReal y, dReal z,
           dReal r, dReal g, dReal b, int rob_id, int wheeltexid, int dir, bool turn_on);
    ~CRobot();
    void step();
    void drawLabel();
    void setSpeed(int i, dReal s); //i = 0,1,2,3
    void setSpeed(dReal vx, dReal vy, dReal vw);
    dReal getSpeed(int i);
    void incSpeed(int i, dReal v);
    void resetSpeeds();
    void resetRobot();
    void getXY(dReal &x, dReal &y);
    dReal getDir();
    dReal getDir(dReal &k);
    void setXY(dReal x, dReal y);
    void setDir(dReal ang);
    int getID();
    PBall *getBall();
    PWorld *getWorld();
};

#define ROBOT_START_Z(cfg) ((cfg)->robotSettings.RobotHeight * 0.5 + (cfg)->robotSettings.WheelRadius * 1 + (cfg)->robotSettings.BottomHeight)

#endif // ROBOT_H
