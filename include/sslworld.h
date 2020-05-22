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

#ifndef SSLWORLD_H
#define SSLWORLD_H


#include <QGLWidget>
#include <QObject>
#include <QUdpSocket>
#include <QList>


#include "graphics.h"
#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"

#include "net/robocup_ssl_server.h"

#include "robot.h"
#include "configwidget.h"

#include "config.h"
#include "speed_estimator.h"
#define WALL_COUNT 16

class RobotsFormation;
class SendingPacket {
    public:
    SendingPacket(fira_message::sim_to_ref::Environment* _packet,int _t);
    fira_message::sim_to_ref::Environment* packet;
    int t;
};

class SSLWorld : public QObject
{
    Q_OBJECT
private:
    QGLWidget* m_parent;
    int frame_num;
    dReal last_dt;
    QList<SendingPacket*> sendQueue;
    char *in_buffer;
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    int time_before = 0;
    int time_after = 0;
    int steps, steps_super;
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT]{};

    void getValidPosition(dReal &x, dReal &y, uint32_t max);

public:    
    dReal customDT;
    int goals_yellow = 0;
    int goals_blue = 0;
    bool isGLEnabled;
    SSLWorld(QGLWidget* parent, ConfigWidget* _cfg, RobotsFormation *form);
    ~SSLWorld() override;
    void glinit();
    void simStep(dReal dt=-1);
    void step(dReal dt=-1);
    void posProcess();
    fira_message::sim_to_ref::Environment* generatePacket();
    void sendVisionBuffer();
    int  robotIndex(unsigned int robot, int team);
    const dReal* ball_vel;
    const dReal* robot_vel;
    const dReal* robot_angular_vel;

    ConfigWidget* cfg;
    CGraphics* g;
    PWorld* p;
    PBall* ball;
    speedEstimator* ball_speed_estimator;
    speedEstimator* blue_speed_estimator [MAX_ROBOT_COUNT];
    speedEstimator* yellow_speed_estimator [MAX_ROBOT_COUNT];
    PGround* ground;
    PRay* ray;
    PFixedBox* walls[WALL_COUNT]{};
    int selected{};
    bool show3DCursor;
    dReal cursor_x{},cursor_y{},cursor_z{};
    dReal cursor_radius{};
    RoboCupSSLServer *visionServer{};
    QUdpSocket *commandSocket{};
    bool updatedCursor;
    bool withGoalKick = false;
    bool randomStart = true;
    CRobot* robots[MAX_ROBOT_COUNT*2]{};
    QElapsedTimer *timer, *timer_fault, *timer_gonca;
    bool received = true, received_first = false;
    int minute = 0;
    dReal last_speed = 0.0;
    std::pair<float, float> ball_prev_pos = std::pair<float, float>(0.0, 0.0);
public slots:
    void recvActions();
signals:
    void fpsChanged(int newFPS);
};

class RobotsFormation {
    public:
        dReal x[MAX_ROBOT_COUNT]{};
        dReal y[MAX_ROBOT_COUNT]{};
        RobotsFormation(int type, ConfigWidget* _cfg);
        void setAll(const dReal *xx,const dReal *yy);
        void loadFromFile(const QString& filename);
        void resetRobots(CRobot** r,int team);
    private:
        ConfigWidget* cfg;
};

dReal fric(dReal f);

#endif // SSLWORLD_H
