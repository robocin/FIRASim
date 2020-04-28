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

#include "sslworld.h"

#include <QtGlobal>
#include <QtNetwork>

#include <QDebug>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include "logger.h"

#include "command.pb.h"
#include "packet.pb.h"
#include "replacement.pb.h"

using namespace fira_message::sim_to_ref;

#define WHEEL_COUNT 2

SSLWorld *_w;
dReal randn_notrig(dReal mu = 0.0, dReal sigma = 1.0);
dReal randn_trig(dReal mu = 0.0, dReal sigma = 1.0);
dReal rand0_1();

dReal fric(dReal f)
{
    if (f + 1 < 0.001)
        return dInfinity;
    return f;
}

bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/)
{
    //s->id2 is ground
    const dReal *r; //wheels rotation matrix
    //const dReal* p; //wheels rotation matrix
    if ((o1 == s->id1) && (o2 == s->id2))
    {
        r = dBodyGetRotation(dGeomGetBody(o1));
        //p=dGeomGetPosition(o1);//never read
    }
    else if ((o1 == s->id2) && (o2 == s->id1))
    {
        r = dBodyGetRotation(dGeomGetBody(o2));
        //p=dGeomGetPosition(o2);//never read
    }
    else
    {
        //XXX: in this case we dont have the rotation
        //     matrix, thus we must return
        return false;
    }

    s->surface.mode = dContactFDir1 | dContactMu2 | dContactApprox1 | dContactSoftCFM;
    s->surface.mu = fric(_w->cfg->robotSettings.WheelPerpendicularFriction);
    s->surface.mu2 = fric(_w->cfg->robotSettings.WheelTangentFriction);
    s->surface.soft_cfm = 0.002;

    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    dReal l = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    s->fdir1[0] = axis[0] / l;
    s->fdir1[1] = axis[1] / l;
    s->fdir1[2] = 0;
    s->fdir1[3] = 0;
    s->usefdir1 = true;
    return true;
}

bool rayCallback(dGeomID o1, dGeomID o2, PSurface *s, int robots_count)
{
    if (!_w->updatedCursor)
        return false;
    dGeomID obj;
    if (o1 == _w->ray->geom)
        obj = o2;
    else
        obj = o1;
    for (int i = 0; i < robots_count * 2; i++)
    {
        if (_w->robots[i]->chassis->geom == obj || _w->robots[i]->dummy->geom == obj)
        {
            _w->robots[i]->selected = true;
            _w->robots[i]->select_x = s->contactPos[0];
            _w->robots[i]->select_y = s->contactPos[1];
            _w->robots[i]->select_z = s->contactPos[2];
        }
    }
    if (_w->ball->geom == obj)
    {
        _w->selected = -2;
    }
    if (obj == _w->ground->geom)
    {
        _w->cursor_x = s->contactPos[0];
        _w->cursor_y = s->contactPos[1];
        _w->cursor_z = s->contactPos[2];
    }
    return false;
}

bool ballCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/)
{
    if (_w->ball->tag != -1) //spinner adjusting
    {
        dReal x, y, z;
        _w->robots[_w->ball->tag]->chassis->getBodyDirection(x, y, z);
        s->fdir1[0] = x;
        s->fdir1[1] = y;
        s->fdir1[2] = 0;
        s->fdir1[3] = 0;
        s->usefdir1 = true;
        s->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
        s->surface.mu = _w->cfg->BallFriction();
        s->surface.mu2 = 0.5;
        s->surface.soft_cfm = 0.002;
    }
    return true;
}

SSLWorld::SSLWorld(QGLWidget *parent, ConfigWidget *_cfg, RobotsFormation *form)
    : QObject(parent)
{
    isGLEnabled = true;
    customDT = -1;
    _w = this;
    cfg = _cfg;
    m_parent = parent;
    show3DCursor = false;
    updatedCursor = false;
    frame_num = 0;
    last_dt = -1;
    g = new CGraphics(parent);
    g->setSphereQuality(1);
    g->setViewpoint(0, -(cfg->Field_Width() + cfg->Field_Margin() * 2.0f) / 2.0f, 3, 90, -45, 0);
    p = new PWorld(0.05, 9.81f, g, cfg->Robots_Count());
    ball = new PBall(0, 0, 0.5, cfg->BallRadius(), cfg->BallMass(), 1, 0.7, 0);

    ground = new PGround(cfg->Field_Rad(), cfg->Field_Length(), cfg->Field_Width(), cfg->Field_Penalty_Depth(), cfg->Field_Penalty_Width(), cfg->Field_Penalty_Point(), cfg->Field_Line_Width(), 0);
    ray = new PRay(50);

    const double thick = cfg->Wall_Thickness();
    const double increment = thick / 2; //cfg->Field_Margin() + cfg->Field_Referee_Margin() + thick / 2;
    const double pos_x = cfg->Field_Length() / 2.0 + increment;
    const double pos_y = cfg->Field_Width() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;

    const double gthick = cfg->Wall_Thickness();
    const double gpos_x = (cfg->Field_Length() + gthick) / 2.0 + cfg->Goal_Depth();
    const double gpos_y = (cfg->Goal_Width() + gthick) / 2.0;
    const double gpos_z = 0; //cfg->Goal_Height() / 2.0;
    const double gsiz_x = cfg->Goal_Depth() + gthick;
    const double gsiz_y = cfg->Goal_Width();
    const double gsiz_z = siz_z; //cfg->Goal_Height();
    const double gpos2_x = (cfg->Field_Length() + gsiz_x) / 2.0;

    // Bounding walls

    for (auto &w : walls)
        w = new PFixedBox(thick / 2, pos_y, pos_z,
                          siz_x, thick, siz_z,
                          tone, tone, tone);

    walls[0] = new PFixedBox(thick / 2, pos_y, pos_z,
                             siz_x, thick, siz_z,
                             tone, tone, tone);

    walls[1] = new PFixedBox(-thick / 2, -pos_y, pos_z,
                             siz_x, thick, siz_z,
                             tone, tone, tone);

    walls[2] = new PFixedBox(pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                             thick, (siz_y - gsiz_y) / 2, siz_z,
                             tone, tone, tone);

    walls[10] = new PFixedBox(pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                              thick, (siz_y - gsiz_y) / 2, siz_z,
                              tone, tone, tone);

    walls[3] = new PFixedBox(-pos_x, gpos_y + (siz_y - gsiz_y) / 4, pos_z,
                             thick, (siz_y - gsiz_y) / 2, siz_z,
                             tone, tone, tone);

    walls[11] = new PFixedBox(-pos_x, -gpos_y - (siz_y - gsiz_y) / 4, pos_z,
                              thick, (siz_y - gsiz_y) / 2, siz_z,
                              tone, tone, tone);

    // Goal walls
    walls[4] = new PFixedBox(gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z,
                             1, 1, 0);

    walls[5] = new PFixedBox(gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             1, 1, 0);

    walls[6] = new PFixedBox(gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             1, 1, 0);

    walls[7] = new PFixedBox(-gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z,
                             0, 0, 1);

    walls[8] = new PFixedBox(-gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             0, 0, 1);

    walls[9] = new PFixedBox(-gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             0, 0, 1);

    // Corner Wall
    walls[12] = new PFixedBox(-pos_x + gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z,
                              tone, tone, tone);
    walls[12]->setRotation(0, 0, 1, M_PI / 4);

    walls[13] = new PFixedBox(pos_x - gsiz_x / 2.8, pos_y - gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z,
                              tone, tone, tone);
    walls[13]->setRotation(0, 0, 1, -M_PI / 4);

    walls[14] = new PFixedBox(pos_x - gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z,
                              tone, tone, tone);
    walls[14]->setRotation(0, 0, 1, M_PI / 4);

    walls[15] = new PFixedBox(-pos_x + gsiz_x / 2.8, -pos_y + gsiz_x / 2.8, pos_z,
                              gsiz_x, gthick, gsiz_z,
                              tone, tone, tone);
    walls[15]->setRotation(0, 0, 1, -M_PI / 4);

    p->addObject(ground);
    p->addObject(ball);
    p->addObject(ray);
    for (auto &wall : walls)
        p->addObject(wall);
    const int wheeltexid = 4 * cfg->Robots_Count() + 12 + 1; //37 for 6 robots
    srand(static_cast<unsigned>(time(0)));

    cfg->robotSettings = cfg->blueSettings;
    for (int k = 0; k < cfg->Robots_Count() * 2; k++)
    {
        bool turn_on = (k % cfg->Robots_Count() < 3) ? true : false;
        float LO_X = -0.65;
        float LO_Y = -0.55;
        float HI_X = 0.65;
        float HI_Y = 0.55;
        float dir = 1.0;
        // if (k > cfg->Robots_Count())
        // {
        //     cfg->robotSettings = cfg->yellowSettings;
        //     x = form->x[k - cfg->Robots_Count()];
        //     y = -form->y[k - cfg->Robots_Count()];
        //     dir = -1;
        // }
        float x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
        float y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
        x = (k % cfg->Robots_Count() < 3) ? x : 3.0;
        y = (k % cfg->Robots_Count() < 3) ? y : 3.0;
        robots[k] = new CRobot(
            p, ball, cfg,
            x, y, ROBOT_START_Z(cfg),
            ROBOT_GRAY, ROBOT_GRAY, ROBOT_GRAY,
            k + 1, wheeltexid, dir, turn_on);
    }

    p->initAllObjects();

    //Surfaces

    p->createSurface(ray, ground)->callback = rayCallback;
    p->createSurface(ray, ball)->callback = rayCallback;
    for (int k = 0; k < cfg->Robots_Count() * 2; k++)
    {
        p->createSurface(ray, robots[k]->chassis)->callback = rayCallback;
        p->createSurface(ray, robots[k]->dummy)->callback = rayCallback;
    }
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1; // | dContactSlip1;
    ballwithwall.surface.mu = 1;                                  //fric(cfg->ballfriction());
    ballwithwall.surface.bounce = cfg->BallBounce();
    ballwithwall.surface.bounce_vel = cfg->BallBounceVel();
    ballwithwall.surface.slip1 = 0; //cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = p->createSurface(ball, ground);
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    for (auto &wall : walls)
        p->createSurface(ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < 2 * cfg->Robots_Count(); k++)
    {
        p->createSurface(robots[k]->chassis, ground);
        for (auto &wall : walls)
            p->createSurface(robots[k]->chassis, wall);
        p->createSurface(robots[k]->dummy, ball);
        //p->createSurface(robots[k]->chassis,ball);
        for (auto &wheel : robots[k]->wheels)
        {
            p->createSurface(wheel->cyl, ball);
            PSurface *w_g = p->createSurface(wheel->cyl, ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (auto &b : robots[k]->balls)
        {
            //            p->createSurface(b->pBall,ball);
            PSurface *w_g = p->createSurface(b->pBall, ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < 2 * cfg->Robots_Count(); j++)
        {
            if (k != j)
            {
                p->createSurface(robots[k]->dummy, robots[j]->dummy); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
            }
        }
    }
    timer = new QElapsedTimer();
    timer->start();
    in_buffer = new char [65536];
    ball_speed_estimator = new speedEstimator(false, 0.95, 100000);
    for(int i=0;i<cfg->Robots_Count();i++){
        blue_speed_estimator[i] = new speedEstimator(true, 0.95, 100000);
        yellow_speed_estimator[i] = new speedEstimator(true, 0.95, 100000);
    }

    // initialize robot state
    for (int team = 0; team < TEAM_COUNT; ++team)
    {
        for (int i = 0; i < MAX_ROBOT_COUNT; ++i)
        {
            lastInfraredState[team][i] = false;
            lastKickState[team][i] = NO_KICK;
        }
    }
}

int SSLWorld::robotIndex(unsigned int robot, int team)
{
    if (robot >= cfg->Robots_Count())
        return -1;
    return robot + team * cfg->Robots_Count();
}

SSLWorld::~SSLWorld()
{
    delete g;
    delete p;
}

QImage *createBlob(char yb, int i, QImage **res)
{
    *res = new QImage(QString(":/%1%2").arg(yb).arg(i) + QString(".png"));
    return *res;
}

QImage *createNumber(int i, int r, int g, int b, int a)
{
    auto *img = new QImage(32, 32, QImage::Format_ARGB32);
    auto *p = new QPainter();
    QBrush br;
    p->begin(img);
    QColor black(0, 0, 0, 0);
    for (int x = 0; x < img->width(); x++)
    {
        for (int j = 0; j < img->height(); j++)
        {
            img->setPixel(x, j, black.rgba());
        }
    }
    QColor txtcolor(r, g, b, a);
    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(3);
    pen.setBrush(txtcolor);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    p->setPen(pen);
    QFont f;
    f.setBold(true);
    f.setPointSize(26);
    p->setFont(f);
    p->drawText(img->width() / 2 - 15, img->height() / 2 - 15, 30, 30, Qt::AlignCenter, QString("%1").arg(i));
    p->end();
    delete p;
    return img;
}

void SSLWorld::glinit()
{
    g->loadTexture(new QImage(":/grass.png"));

    // Loading Robot textures for each robot
    for (int i = 0; i < cfg->Robots_Count(); i++)
        g->loadTexture(createBlob('b', i, &robots[i]->img));

    for (int i = 0; i < cfg->Robots_Count(); i++)
        g->loadTexture(createBlob('y', i, &robots[cfg->Robots_Count() + i]->img));

    // Creating number textures
    for (int i = 0; i < cfg->Robots_Count(); i++)
        g->loadTexture(createNumber(i, 15, 193, 225, 255));

    for (int i = 0; i < cfg->Robots_Count(); i++)
        g->loadTexture(createNumber(i, 0xff, 0xff, 0, 255));

    // Loading sky textures
    // XXX: for some reason they are loaded twice otherwise the wheel texture is wrong
    for (int i = 0; i < 6; i++)
    {
        g->loadTexture(new QImage(QString(":/sky/neg_%1").arg(i % 3 == 0 ? 'x' : i % 3 == 1 ? 'y' : 'z') + QString(".png")));
        g->loadTexture(new QImage(QString(":/sky/pos_%1").arg(i % 3 == 0 ? 'x' : i % 3 == 1 ? 'y' : 'z') + QString(".png")));
    }

    // The wheel texture
    g->loadTexture(new QImage(":/wheel.png"));

    // Init at last
    p->glinit();
}

void SSLWorld::step(dReal dt)
{
    if (!isGLEnabled)
        g->disableGraphics();
    else
        g->enableGraphics();
    if (customDT > 0)
        dt = customDT;

    const auto ratio = m_parent->devicePixelRatio();
    g->initScene(m_parent->width() * ratio, m_parent->height() * ratio, 0, 0.7, 1);
    // Pq ele faz isso 5 vezes?
    // - Talvez mais precisao (Ele sempre faz um step de dt*0.2 )
    for (int kk = 0; kk < 5; kk++)
    {
        const dReal *ballvel = dBodyGetLinearVel(ball->body);
        // Norma do vetor velocidade da bola
        dReal ballspeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballspeed = sqrt(ballspeed);
        dReal ballfx = 0, ballfy = 0, ballfz = 0;
        dReal balltx = 0, ballty = 0, balltz = 0;
        if (ballspeed < 0.01)
        {
            ; //const dReal* ballAngVel = dBodyGetAngularVel(ball->body);
            //TODO: what was supposed to be here?
        }
        else
        {
            // Velocidade real  normalizada (com atrito envolvido) da bola
            dReal fk = cfg->BallFriction() * cfg->BallMass() * cfg->Gravity();
            ballfx = -fk * ballvel[0] / ballspeed;
            ballfy = -fk * ballvel[1] / ballspeed;
            ballfz = -fk * ballvel[2] / ballspeed;
            balltx = -ballfy * cfg->BallRadius();
            ballty = ballfx * cfg->BallRadius();
            balltz = 0;
            dBodyAddTorque(ball->body, balltx, ballty, balltz);
        }
        dBodyAddForce(ball->body, ballfx, ballfy, ballfz);
        if (dt == 0)
            dt = last_dt;
        else
            last_dt = dt;

        selected = -1;
        p->step(dt * 0.2);
    }

    int best_k = -1;
    dReal best_dist = 1e8;
    dReal xyz[3], hpr[3];
    if (selected == -2)
    {
        best_k = -2;
        dReal bx, by, bz;
        ball->getBodyPosition(bx, by, bz);
        g->getViewpoint(xyz, hpr);
        best_dist = (bx - xyz[0]) * (bx - xyz[0]) + (by - xyz[1]) * (by - xyz[1]) + (bz - xyz[2]) * (bz - xyz[2]);
    }
    for (int k = 0; k < cfg->Robots_Count() * 2; k++)
    {
        if (robots[k]->selected)
        {
            g->getViewpoint(xyz, hpr);
            dReal dist = (robots[k]->select_x - xyz[0]) * (robots[k]->select_x - xyz[0]) + (robots[k]->select_y - xyz[1]) * (robots[k]->select_y - xyz[1]) + (robots[k]->select_z - xyz[2]) * (robots[k]->select_z - xyz[2]);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_k = k;
            }
        }
        robots[k]->chassis->setColor(ROBOT_GRAY, ROBOT_GRAY, ROBOT_GRAY);
    }
    if (best_k >= 0)
        robots[best_k]->chassis->setColor(ROBOT_GRAY * 2, ROBOT_GRAY * 1.5, ROBOT_GRAY * 1.5);
    selected = best_k;
    ball->tag = -1;
    for (int k = 0; k < cfg->Robots_Count() * 2; k++)
    {
        robots[k]->step();
        robots[k]->selected = false;
    }
    p->draw();
    //g->drawSkybox(31,32,33,34,35,36);
    g->drawSkybox(4 * cfg->Robots_Count() + 6 + 1,  //31 for 6 robot
                  4 * cfg->Robots_Count() + 6 + 2,  //32 for 6 robot
                  4 * cfg->Robots_Count() + 6 + 3,  //33 for 6 robot
                  4 * cfg->Robots_Count() + 6 + 4,  //34 for 6 robot
                  4 * cfg->Robots_Count() + 6 + 5,  //31 for 6 robot
                  4 * cfg->Robots_Count() + 6 + 6); //36 for 6 robot

    dMatrix3 R;

    if (g->isGraphicsEnabled())
        if (show3DCursor)
        {
            dRFromAxisAndAngle(R, 0, 0, 1, 0);
            g->setColor(1, 0.9, 0.2, 0.5);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            g->drawCircle(cursor_x, cursor_y, 0.001, cursor_radius);
            glDisable(GL_BLEND);
        }
    //for (int k=0;k<10;k++) robots[k]->drawLabel();

    g->finalizeScene();
  

    sendVisionBuffer();
    posProcess();
    frame_num++;
}

void SSLWorld::recvActions()
{
    QHostAddress sender;
    quint16 port;
    Packet packet;
    while (commandSocket->hasPendingDatagrams())
    {
        qint64 size = commandSocket->readDatagram(in_buffer, 65536, &sender, &port);
        if (size > 0)
        {
            packet.ParseFromArray(in_buffer, static_cast<int>(size));
            if (packet.has_cmd())
            {
                for (const auto &robot_cmd : packet.cmd().robot_commands())
                {
                    int id = robotIndex(robot_cmd.id(), robot_cmd.yellowteam());
                    if ((id < 0) || (id >= cfg->Robots_Count() * 2))
                        continue;
                    robots[id]->setSpeed(0, -1 * robot_cmd.wheel_left());
                    robots[id]->setSpeed(1, robot_cmd.wheel_right());
                }
            }
            if (packet.has_replace())
            {
                for (const auto &replace : packet.replace().robots())
                {
                    int id = robotIndex(replace.position().robot_id(), replace.yellowteam());
                    if ((id < 0) || (id >= cfg->Robots_Count() * 2))
                        continue;
                    robots[id]->setXY(replace.position().x(), replace.position().y());
                    robots[id]->setDir(replace.position().orientation());
                    robots[id]->on = replace.turnon();
                }
                if (packet.replace().has_ball())
                {
                    dReal x = 0, y = 0, z = 0, vx = 0, vy = 0;
                    ball->getBodyPosition(x, y, z);
                    const auto vel_vec = dBodyGetLinearVel(ball->body);
                    vx = vel_vec[0];
                    vy = vel_vec[1];

                    x = packet.replace().ball().x();
                    y = packet.replace().ball().y();
                    vx = packet.replace().ball().vx();
                    vy = packet.replace().ball().vy();

                    ball->setBodyPosition(x, y, cfg->BallRadius() * 1.2);
                    dBodySetLinearVel(ball->body, vx, vy, 0);
                    dBodySetAngularVel(ball->body, 0, 0, 0);
                }
            }
        }
    }
}

dReal normalizeAngle(dReal a)
{
    if (a > 180)
        return -360 + a;
    if (a < -180)
        return 360 + a;
    return a;
}

Environment *SSLWorld::generatePacket()
{
    int t = timer->elapsed();
    auto* env = new Environment;
    dReal x,y,z,dir,k;
    ball->getBodyPosition(x,y,z); 
    //Estimating Ball Speed  

    //Ball Pose 
    dReal ball_pose[3];
    ball_pose[0] = x;
    ball_pose[1] = y;
    ball_pose[2] = 0.0; //Ball's Angle Not considered
    dReal ball_vel[3] = {0.0};
    ball_speed_estimator->estimateSpeed((double)(t), ball_pose, ball_vel);
    //Ball speed stored in vall_vel. Remember that the sign for linear speed is changed.
    dReal dev_x = cfg->noiseDeviation_x();
    dReal dev_y = cfg->noiseDeviation_y();
    dReal dev_a = cfg->noiseDeviation_angle();
    if (!cfg->noise())
    {
        dev_x = 0;
        dev_y = 0;
        dev_a = 0;
    }
    if (!cfg->vanishing() || (rand0_1() > cfg->ball_vanishing()))
    {
        auto *vball = env->mutable_frame()->mutable_ball();
        vball->set_x(randn_notrig(x, dev_x));
        vball->set_y(randn_notrig(y, dev_y));
        vball->set_z(z);
    }
    for (uint32_t i = 0; i < cfg->Robots_Count() * 2; i++)
    {
        if (!cfg->vanishing() || (rand0_1() > cfg->blue_team_vanishing()))
        {
            if (!robots[i]->on)
                continue;
            robots[i]->getXY(x, y);
            dir = robots[i]->getDir(k);
            //Estimating speeds for robots
            //Robot Pose
            dReal robot_pose[3];
            robot_pose[0] = x;
            robot_pose[1] = y;
            robot_pose[2] = (normalizeAngle(dir)*M_PI/180.0);
            dReal robot_vel[3]={0.0};
            if(i<cfg->Robots_Count()) {
                blue_speed_estimator[i]->estimateSpeed((double)t, robot_pose, robot_vel);
            } else{
                yellow_speed_estimator[i-cfg->Robots_Count()]->estimateSpeed((double) t, robot_pose, robot_vel);
            }
            //Robot speed stored in robot_vel. Remember that the sign for linear speed is changed.

            // reset when the robot has turned over
            if (cfg->ResetTurnOver() && k < 0.9)
            {
                robots[i]->resetRobot();
            }
            fira_message::Robot *rob;
            if (i < cfg->Robots_Count())
                rob = env->mutable_frame()->add_robots_blue();
            else
                rob = env->mutable_frame()->add_robots_yellow();
            rob->set_robot_id(i - cfg->Robots_Count());
            if (i >= cfg->Robots_Count())
                rob->set_robot_id(i - cfg->Robots_Count());
            else
                rob->set_robot_id(i);
            rob->set_x(randn_notrig(x, dev_x));
            rob->set_y(randn_notrig(y, dev_y));
            rob->set_orientation(normalizeAngle(randn_notrig(dir, dev_a)) * M_PI / 180.0);
        }
    }
    fira_message::Field *field = env->mutable_field();
    field->set_width(cfg->Field_Width());
    field->set_length(cfg->Field_Length());
    field->set_goal_depth(cfg->Goal_Depth());
    field->set_goal_width(cfg->Goal_Width());
    env->set_step(timer->elapsed());
    return env;
}

SendingPacket::SendingPacket(fira_message::sim_to_ref::Environment *_packet, int _t)
{
    packet = _packet;
    t = _t;
}

void SSLWorld::sendVisionBuffer()
{
    int t = timer->elapsed();
    sendQueue.push_back(new SendingPacket(generatePacket(), t));
    while (t - sendQueue.front()->t >= cfg->sendDelay())
    {
        Environment *packet = sendQueue.front()->packet;
        delete sendQueue.front();
        sendQueue.pop_front();
        visionServer->send(*packet);
        delete packet;
        if (sendQueue.isEmpty())
            break;
    }
}

void SSLWorld::posProcess()
{
    bool is_goal = false;
    dReal bx, by, bz;
    ball->getBodyPosition(bx, by, bz);
    if (bx > 0.75 && abs(by) < 0.4)
    {
        goals_blue++;
        is_goal = true;
    }
    else if (bx < -0.75 && abs(by) < 0.4)
    {
        goals_yellow++;
        is_goal = true;
    }
    // if(bx < -0.6 && abs(by < 0.35))

    time_before = time_after;
    time_after = timer->elapsed()/300000;
    bool end_time = time_after != time_before;
    if (is_goal || end_time)
    {
        float LO_X = -0.65;
        float LO_Y = -0.55;
        float HI_X = 0.65;
        float HI_Y = 0.55;
        srand(static_cast<unsigned>(time(0)));
        float x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
        float y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
        ball->setBodyPosition(x, y, 0);
        for (uint32_t i = 0; i < cfg->Robots_Count() * 2; i++)
        {
            x = LO_X + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_X - LO_X)));
            y = LO_Y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (HI_Y - LO_Y)));
            if (!robots[i]->on)
                continue;
            robots[i]->setXY(x, y);
        }
        timer->restart();
        time_before = time_after = 0;
    }
}

void RobotsFormation::setAll(const dReal *xx, const dReal *yy)
{
    for (int i = 0; i < cfg->Robots_Count(); i++)
    {
        x[i] = xx[i];
        y[i] = yy[i];
    }
}

RobotsFormation::RobotsFormation(int type, ConfigWidget *_cfg) : cfg(_cfg)
{
    if (type == 0)
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.2, 1.0, 1.0, 1.0, 0.33, 1.22,
                                           3, 3.2, 3.4, 3.6, 3.8, 4.0};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.0, -0.75, 0.0, 0.75, 0.25, 0.0,
                                           1, 1, 1, 1, 1, 1};
        setAll(teamPosX, teamPosY);
    }
    if (type == 1) // formation 1
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {1.5, 1.5, 1.5, 0.55, 2.5, 3.6,
                                           3.2, 3.2, 3.2, 3.2, 3.2, 3.2};
        dReal teamPosY[MAX_ROBOT_COUNT] = {1.12, 0.0, -1.12, 0.0, 0.0, 0.0,
                                           0.75, -0.75, 1.5, -1.5, 2.25, -2.25};
        setAll(teamPosX, teamPosY);
    }
    if (type == 2) // formation 2
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {4.2, 3.40, 3.40, 0.7, 0.7, 0.7,
                                           2, 2, 2, 2, 2, 2};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.0, -0.20, 0.20, 0.0, 2.25, -2.25,
                                           0.75, -0.75, 1.5, -1.5, 2.25, -2.25};
        setAll(teamPosX, teamPosY);
    }
    if (type == 3) // outside field
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.4, 0.8, 1.2, 1.6, 2.0, 2.4,
                                           2.8, 3.2, 3.6, 4.0, 4.4, 4.8};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-4.0, -4.0, -4.0, -4.0, -4.0, -4.0,
                                           -4.0, -4.0, -4.0, -4.0, -4.0, -4.0};
        setAll(teamPosX, teamPosY);
    }
    if (type == 4)
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.8, 2.5, 2.5, 0.8, 0.8, 1.1, 3, 3.2, 3.4, 3.6, 3.8, 4.0};
        dReal teamPosY[MAX_ROBOT_COUNT] = {5 + 0.0, 5 - 0.3, 5 + 0.3, 5 + 0.0, 5 + 1.5, 5.5, 1, 1, 1, 1, 1, 1};
        setAll(teamPosX, teamPosY);
    }
    if (type == -1) // outside
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.4, 0.8, 1.2, 1.6, 2.0, 2.4,
                                           2.8, 3.2, 3.6, 4.0, 4.4, 4.8};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-3.4, -3.4, -3.4, -3.4, -3.4, -3.4,
                                           -3.4, -3.4, -3.4, -3.4, -3.4, -3.4};
        setAll(teamPosX, teamPosY);
    }
}

void RobotsFormation::loadFromFile(const QString &filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file);
    int k;
    for (k = 0; k < cfg->Robots_Count(); k++)
        x[k] = y[k] = 0;
    k = 0;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList list = line.split(",");
        if (list.count() >= 2)
        {
            x[k] = list[0].toFloat();
            y[k] = list[1].toFloat();
        }
        else if (list.count() == 1)
        {
            x[k] = list[0].toFloat();
        }
        if (k == cfg->Robots_Count() - 1)
            break;
        k++;
    }
}

void RobotsFormation::resetRobots(CRobot **r, int team)
{
    dReal dir = -1;
    if (team == 1)
        dir = 1;
    for (int k = 0; k < cfg->Robots_Count(); k++)
    {
        r[k + team * cfg->Robots_Count()]->setXY(x[k] * dir, y[k]);
        r[k + team * cfg->Robots_Count()]->resetRobot();
    }
}

//// Copy & pasted from http://www.dreamincode.net/code/snippet1446.htm
/******************************************************************************/
/* randn()
 *
 * Normally (Gaussian) distributed random numbers, using the Box-Muller
 * transformation.  This transformation takes two uniformly distributed deviates
 * within the unit circle, and transforms them into two independently
 * distributed normal deviates.  Utilizes the internal rand() function; this can
 * easily be changed to use a better and faster RNG.
 *
 * The parameters passed to the function are the mean and standard deviation of
 * the desired distribution.  The default values used, when no arguments are
 * passed, are 0 and 1 - the standard normal distribution.
 *
 *
 * Two functions are provided:
 *
 * The first uses the so-called polar version of the B-M transformation, using
 * multiple calls to a uniform RNG to ensure the initial deviates are within the
 * unit circle.  This avoids making any costly trigonometric function calls.
 *
 * The second makes only a single set of calls to the RNG, and calculates a
 * position within the unit circle with two trigonometric function calls.
 *
 * The polar version is generally superior in terms of speed; however, on some
 * systems, the optimization of the math libraries may result in better
 * performance of the second.  Try it out on the target system to see which
 * works best for you.  On my test machine (Athlon 3800+), the non-trig version
 * runs at about 3x10^6 calls/s; while the trig version runs at about
 * 1.8x10^6 calls/s (-O2 optimization).
 *
 *
 * Example calls:
 * randn_notrig();	//returns normal deviate with mean=0.0, std. deviation=1.0
 * randn_notrig(5.2,3.0);	//returns deviate with mean=5.2, std. deviation=3.0
 *
 *
 * Dependencies - requires <cmath> for the sqrt(), sin(), and cos() calls, and a
 * #defined value for PI.
 */

/******************************************************************************/
//	"Polar" version without trigonometric calls
dReal randn_notrig(dReal mu, dReal sigma)
{
    if (sigma == 0)
        return mu;
    static bool deviateAvailable = false; //	flag
    static dReal storedDeviate;           //	deviate from previous calculation
    dReal polar, rsquared, var1, var2;

    //	If no deviate has been stored, the polar Box-Muller transformation is
    //	performed, producing two independent normally-distributed random
    //	deviates.  One is stored for the next round, and one is returned.
    if (!deviateAvailable)
    {

        //	choose pairs of uniformly distributed deviates, discarding those
        //	that don't fall within the unit circle
        do
        {
            var1 = 2.0 * (dReal(rand()) / dReal(RAND_MAX)) - 1.0;
            var2 = 2.0 * (dReal(rand()) / dReal(RAND_MAX)) - 1.0;
            rsquared = var1 * var1 + var2 * var2;
        } while (rsquared >= 1.0 || rsquared == 0.0);

        //	calculate polar tranformation for each deviate
        polar = sqrt(-2.0 * log(rsquared) / rsquared);

        //	store first deviate and set flag
        storedDeviate = var1 * polar;
        deviateAvailable = true;

        //	return second deviate
        return var2 * polar * sigma + mu;
    }

    //	If a deviate is available from a previous call to this function, it is
    //	returned, and the flag is set to false.
    else
    {
        deviateAvailable = false;
        return storedDeviate * sigma + mu;
    }
}

/******************************************************************************/
//	Standard version with trigonometric calls
#define PI 3.14159265358979323846

dReal randn_trig(dReal mu, dReal sigma)
{
    static bool deviateAvailable = false; //	flag
    static dReal storedDeviate;           //	deviate from previous calculation
    dReal dist, angle;

    //	If no deviate has been stored, the standard Box-Muller transformation is
    //	performed, producing two independent normally-distributed random
    //	deviates.  One is stored for the next round, and one is returned.
    if (!deviateAvailable)
    {

        //	choose a pair of uniformly distributed deviates, one for the
        //	distance and one for the angle, and perform transformations
        dist = sqrt(-2.0 * log(dReal(rand()) / dReal(RAND_MAX)));
        angle = 2.0 * PI * (dReal(rand()) / dReal(RAND_MAX));

        //	calculate and store first deviate and set flag
        storedDeviate = dist * cos(angle);
        deviateAvailable = true;

        //	calculate return second deviate
        return dist * sin(angle) * sigma + mu;
    }

    //	If a deviate is available from a previous call to this function, it is
    //	returned, and the flag is set to false.
    else
    {
        deviateAvailable = false;
        return storedDeviate * sigma + mu;
    }
}

dReal rand0_1()
{
    return (dReal)(rand()) / (dReal)(RAND_MAX);
}
