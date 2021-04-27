#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define PI 3.14159

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
const unsigned CHECK_POINT_RADIUS = 600;
const unsigned VEHICLE_RADIUS = 400;

float RadiansToDegrees(float radians) 
{
    return radians*(180.0f/PI);
}

float DegreesToRadians(float degrees) 
{
    return degrees*(PI/180.0f);
}

struct Vec2
{
    int x;
    int y;

    Vec2() : x(0), y(0) {}
    Vec2(int _x, int _y) : x(_x), y(_y) {}

    Vec2 operator-(const Vec2& rhs) const
    {
        Vec2 res;
        res.x = x-rhs.x;
        res.y = y-rhs.y;

        return res;
    }

    Vec2 operator+(const Vec2& rhs) const
    {
        Vec2 res;
        res.x = x+rhs.x;
        res.y = y+rhs.y;

        return res;
    }

    float Length() const
    {
        return sqrtf(float(x*x+y*y));
    }

    void Scale(int value)
    {
        float len = Length();
        float mul = float(value)/len;

        x = int(x*mul);
        y = int(y*mul);
    }

    int Dot(const Vec2& rhs) const
    {
        return x*rhs.x+y*rhs.y;
    }

    int GetAngle() const
    {
        float angle = x != 0 ? atan2(float(y), float(x)): 0.0f;
        return int(RadiansToDegrees(angle));
    }

    int GetAngleBetween(const Vec2& rhs) const
    {
        float angle0 = x != 0 ? atan2(float(y), float(x)) : 0.0f;
        float angle1 = rhs.x != 0 ? atan2(float(rhs.y), float(rhs.x)) : 0.0f;

        float angle = angle0-angle1;

        if(angle > PI) angle -=2.0f*PI;
        if(angle <-PI) angle += 2.0F*PI;

        return int(RadiansToDegrees(angle));
    }

    void Rotate(int degrees)
    {
        float radians = DegreesToRadians(float(degrees));
        float cs = cos(radians);
        float sn = sin(radians);

        float xaux = x*cs-y*sn;
        float yaux = x*sn+y*cs;

        x = int(xaux);
        y = int(yaux);
    }

    bool operator==(const Vec2& rhs) const
    {
        return x == rhs.x && y == rhs.y;
    }

    friend ostream& operator<< (ostream& os, const Vec2& vec)
    {
        os << "(" << vec.x << ", " << vec.y << ")";
        return os;
    }
};

struct VehicleInfo
{
    Vec2 pos;
    Vec2 prevPos;
    Vec2 nextPos;
    Vec2 speed;

    void Update(int x, int y)
    {
        prevPos = pos;
        pos     = Vec2(x, y);
        speed   = pos-prevPos;
        nextPos = pos+speed;
    }

    bool WillCollide(const VehicleInfo& vehicle) const
    {
        Vec2 diff = nextPos-vehicle.nextPos;
        return diff.Dot(diff) < (VEHICLE_RADIUS*VEHICLE_RADIUS*4);
    }
};

VehicleInfo player;
VehicleInfo boss;

struct StateInfo
{
    Vec2 checkPointPos;
    int checkPointAngle = 0;
    int prevCheckPointAngle = 0;
    int angleSpeed = 0;
    bool boostDone = false;
    int thrust = 100;
    Vec2 lastMoveDir;
    int inertiaAngle = 0;

    void UpdatePosAngle(const Vec2& pos, int angle)
    {
        prevCheckPointAngle = checkPointAngle;
        checkPointPos = pos;
        checkPointAngle = angle;
        angleSpeed = checkPointAngle-prevCheckPointAngle;

        inertiaAngle = player.speed.GetAngleBetween(lastMoveDir);        
    }

    void SendAction(const Vec2& target, int thrust, bool boost)
    {
        if(boost)
        {
            cout << target.x << " " << target.y << " BOOST" << endl;

            boostDone = true;
        }
        else
        {
            cerr << "Correction with inertia angle " << inertiaAngle << endl;

            Vec2 targetDir = target-player.pos;

            if(abs(inertiaAngle) < 90)
            {
                targetDir.Rotate(-inertiaAngle);
            }

            Vec2 corrected = player.pos+targetDir;

            cout << corrected.x << " " << corrected.y << " " << thrust << endl;

            thrust = thrust;
        }

        lastMoveDir = (target-player.pos);
    }
};

StateInfo state;


bool ShouldTryCollide()
{
    const float MIN_VEHICLE_DISTANCE = VEHICLE_RADIUS*2.5;

    float playerSpeed = player.speed.Length();
    float bossSpeed = boss.speed.Length();

    // My speed is bigger
    bool collide = bossSpeed < playerSpeed;
    

    // we are near    
    collide = collide && (player.nextPos-boss.nextPos).Length() < MIN_VEHICLE_DISTANCE;

    // he is in front of my direction
    collide = collide && (state.checkPointPos-player.pos).Dot(boss.pos-player.pos) > 0;
    
    //collide = collide && (state.checkPointPos-player.nextPos).Length() > CHECK_POINT_RADIUS*2;

    if(collide)
    {
        cerr << "collide speeds " << bossSpeed << " " << playerSpeed << endl;
        cerr << "collide distance " << (player.nextPos-boss.nextPos).Length() << " " << MIN_VEHICLE_DISTANCE << endl;
    }

    return collide;
}

void ComputeNextTarget(Vec2& target, bool& boost, int& thrust) 
{
    target = state.checkPointPos;
    boost = false;
        
    // TODO: Si checkpoint angle crece menos thurst si decrece más thurst
    
    if(state.checkPointAngle > 90 || state.checkPointAngle < -90)
    {
        cerr << "Too much angle thrust 0" << endl;
        thrust  = 0;
    }
    else if((target-player.nextPos).Length() < CHECK_POINT_RADIUS)
    {
        cerr << "Arriving checkpoint thrust 0" << endl;

        thrust = 0;
    }
    else if(ShouldTryCollide())  
    {
        if(!state.boostDone)
        {
            cerr << "Will collide with boost " << player.pos << " " << boss.nextPos << endl;
            boost = true;
            target = boss.nextPos;
        }
        else
        {
            cerr << "Will collide without boost " << player.pos << " " << boss.nextPos << endl;
            target = boss.nextPos;
            thrust = 100;
        }
    }
    else 
    {   
        
        int angleTime = abs(state.angleSpeed) > 0 ? abs(state.checkPointAngle)/abs(state.angleSpeed) : std::numeric_limits<int>::max();

        float speed = player.speed.Length();
        float distance = (state.checkPointPos-player.pos).Length();
        int linearTime = speed > 0.0f ? int(distance/speed) : std::numeric_limits<int>::max();

        
        cerr << "Times " << linearTime << " " << angleTime << endl;

        // TODO: Use inertia angle

        if(linearTime < angleTime && abs(state.checkPointAngle) > 5 && state.angleSpeed > 0)
        {
            cerr << "linear speed greater than angular thrust 0" << endl;
            thrust = 0; // std::max(state.thrust-33, 0);
        }    
        else if(state.angleSpeed == 0 && abs(state.checkPointAngle) > 5)
        {
            cerr << "can't force angle to 0 thrust 0 " << state.angleSpeed << " " << state.checkPointAngle << endl;
            thrust = 0; // std::max(state.thrust-10, 0);
        }
        
        if(!state.boostDone &&  state.checkPointAngle == 0 && distance > 3000)
        {
            cerr << "good moment to boost " << endl;
            boost = true;
        }
        else
        {
            cerr << "acceleration " << endl;
            thrust = 100; 
        } 
    }

    cerr << "Checkpoint angle " << state.checkPointAngle << endl;
    cerr << "current speed " << player.speed.Length() << endl;
    cerr << "boss speed " << boss.speed.Length() << endl;
    cerr << (state.boostDone ? "boost done" : "boost not done") << endl;
}

int main()
{

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;                

        // \todo compensar la inercia

        player.Update(x, y);
        boss.Update(opponentX, opponentY);
        state.UpdatePosAngle(Vec2(nextCheckpointX, nextCheckpointY), nextCheckpointAngle);

        Vec2 target; 
        int thrust = 0;
        bool boost = false;   

        ComputeNextTarget(target, boost, thrust);        

        state.SendAction(target, thrust, boost);                
    }
}
